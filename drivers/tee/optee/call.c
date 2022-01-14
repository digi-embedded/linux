// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Linaro Limited
 */
#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include "optee_private.h"
#include "optee_smc.h"
#define CREATE_TRACE_POINTS
#include "optee_trace.h"

/* Requires the filpstate mutex to be held */
static struct optee_session *find_session(struct optee_context_data *ctxdata,
					  u32 session_id)
{
	struct optee_session *sess;

	list_for_each_entry(sess, &ctxdata->sess_list, list_node)
		if (sess->session_id == session_id)
			return sess;

	return NULL;
}

static void param_clear_ocall(struct tee_param *ocall)
{
	if (ocall)
		memset(&ocall->u, 0, sizeof(ocall->u));
}

static u64 param_get_ocall_func(struct tee_param *param)
{
	return TEE_IOCTL_OCALL_GET_FUNC(param->u.value.a);
}

/* Requires @sem in the parent struct optee_session to be held */
static int verify_ocall_request(u32 num_params, struct optee_call_ctx *call_ctx)
{
	struct optee_msg_arg *arg = call_ctx->rpc_arg;

	switch (arg->cmd) {
	case OPTEE_MSG_RPC_CMD_OCALL:
		/* 'num_params' is checked later */

		/* These parameters carry the OCALL descriptors */
		if (arg->num_params < 2 ||
		    arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INOUT ||
		    arg->params[1].attr != OPTEE_MSG_ATTR_TYPE_VALUE_INPUT ||
		    arg->params[0].u.value.a > U32_MAX ||  /* OCALL Cmd Id */
		    arg->params[1].u.value.c != 0)  /* TA UUID (128 bytes) */
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Requires @sem in the parent struct optee_session to be held */
static int verify_ocall_reply(u64 func, struct tee_param *params,
			      u32 num_params, struct optee_call_ctx *call_ctx)
{
	size_t n;

	switch (func) {
	case TEE_IOCTL_OCALL_CMD_INVOKE:
		if (call_ctx->rpc_arg->cmd != OPTEE_MSG_RPC_CMD_OCALL)
			return -EINVAL;

		/* Skip the loop below */
		return 0;
	default:
		return -EINVAL;
	}

	/* The remaining parameters are unused */
	for (n = 1; n < num_params; n++)
		if (params[n].attr != TEE_IOCTL_PARAM_ATTR_TYPE_NONE)
			return -EINVAL;

	return 0;
}

/* Requires @sem in the parent struct optee_session to be held */
static void process_ocall_memrefs(struct optee_msg_param *params,
				  u32 num_params, bool increment)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		struct tee_shm *shm;
		const struct optee_msg_param *mp = params + n;
		u32 attr = mp->attr & OPTEE_MSG_ATTR_TYPE_MASK;

		switch (attr) {
		case OPTEE_MSG_ATTR_TYPE_RMEM_INPUT:
		case OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT:
		case OPTEE_MSG_ATTR_TYPE_RMEM_INOUT:
			shm = (struct tee_shm *)(uintptr_t)mp->u.rmem.shm_ref;
			break;
		default:
			shm = NULL;
			break;
		}

		if (!shm)
			continue;

		if (increment)
			tee_shm_get(shm);
		else
			tee_shm_put(shm);
	}
}

/*
 * Requires @sem in the parent struct optee_session to be held (if OCALLs are
 * expected)
 */
static void call_prologue(struct optee_call_ctx *call_ctx)
{
	struct optee *optee = tee_get_drvdata(call_ctx->ctx->teedev);

	/* Initialize waiter */
	optee_cq_wait_init(&optee->call_queue, &call_ctx->waiter);
}

/*
 * Requires @sem in the parent struct optee_session to be held (if OCALLs are
 * expected)
 */
static void call_epilogue(struct optee_call_ctx *call_ctx)
{
	struct optee *optee = tee_get_drvdata(call_ctx->ctx->teedev);

	optee_rpc_finalize_call(call_ctx);

	/*
	 * We're done with our thread in secure world, if there's any
	 * thread waiters wake up one.
	 */
	optee_cq_wait_final(&optee->call_queue, &call_ctx->waiter);
}

/* Requires @sem in the parent struct optee_session to be held */
static int process_ocall_request(struct tee_param *params, u32 num_params,
				 struct tee_param *ocall,
				 struct optee_call_ctx *call_ctx)
{
	u32 cmd_id;
	struct optee_msg_param *msg_param;
	u32 msg_num_params;
	int rc = 0;

	/*
	 * Points to the octets of the UUID corresponding to the TA requesting
	 * the OCALL, if applicable for this call.
	 */
	void *clnt_id;

	rc = verify_ocall_request(num_params, call_ctx);
	if (rc)
		goto exit_set_ret;

	/*
	 * Clear out the parameters of the original function invocation. The
	 * original contents are backed up in call_ctx->msg_arg and will be
	 * restored elsewhere once the OCALL is over.
	 */
	memset(params, 0, num_params * sizeof(*params));

	/* Set up the OCALL request */
	switch (call_ctx->rpc_arg->cmd) {
	case OPTEE_MSG_RPC_CMD_OCALL:
		/* -2 here and +2 below to skip the OCALL descriptors */
		msg_num_params = call_ctx->rpc_arg->num_params - 2;
		if (num_params < msg_num_params) {
			rc = -EINVAL;
			goto exit_set_ret;
		}

		msg_param = call_ctx->rpc_arg->params + 2;
		rc = optee_from_msg_param(params, msg_num_params, msg_param);
		if (rc)
			goto exit_set_ret;

		process_ocall_memrefs(msg_param, msg_num_params, true);
		call_ctx->rpc_must_release = true;

		cmd_id = (u32)call_ctx->rpc_arg->params[0].u.value.a;
		ocall->u.value.a =
			TEE_IOCTL_OCALL_MAKE_PAIR(TEE_IOCTL_OCALL_CMD_INVOKE,
						  cmd_id);

		clnt_id = &call_ctx->rpc_arg->params[1].u.value;
		memcpy(&ocall->u.value.b, clnt_id, TEE_IOCTL_UUID_LEN);
		break;
	default:
		/* NOT REACHED */
		rc = -EINVAL;
		goto exit_set_ret;
	}

	return rc;

exit_set_ret:
	call_ctx->rpc_arg->ret = TEEC_ERROR_BAD_PARAMETERS;
	call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;
	return rc;
}

/* Requires @sem in the parent struct optee_session to be held */
static int process_ocall_reply(u32 ret, u32 ret_origin,
			       struct tee_param *params, u32 num_params,
			       struct tee_param *ocall,
			       struct optee_call_ctx *call_ctx)
{
	const u64 func = param_get_ocall_func(ocall);
	struct optee_msg_param *msg_param;
	u32 msg_num_params;
	int rc = 0;

	rc = verify_ocall_reply(func, params, num_params, call_ctx);
	if (rc)
		goto exit_set_ret;

	switch (func) {
	case TEE_IOCTL_OCALL_CMD_INVOKE:
		/* -2 here and +2 below to skip the OCALL descriptors */
		msg_num_params = call_ctx->rpc_arg->num_params - 2;
		if (num_params < msg_num_params) {
			rc = -EINVAL;
			goto exit_set_ret;
		}

		msg_param = call_ctx->rpc_arg->params + 2;
		rc = optee_to_msg_param(msg_param, msg_num_params, params);
		if (rc)
			goto exit_set_ret;

		process_ocall_memrefs(msg_param, msg_num_params, false);
		call_ctx->rpc_must_release = false;

		call_ctx->rpc_arg->params[0].u.value.b = ret;
		call_ctx->rpc_arg->params[0].u.value.c = ret_origin;
		break;
	default:
		rc = -EINVAL;
		goto exit_set_ret;
	}

	call_ctx->rpc_arg->ret = TEEC_SUCCESS;
	call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;

	return rc;

exit_set_ret:
	call_ctx->rpc_arg->ret = TEEC_ERROR_BAD_PARAMETERS;
	call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;
	return rc;
}

static void clear_call_ctx(struct optee_call_ctx *call_ctx)
{
	memset(call_ctx, 0, sizeof(*call_ctx));
}

/**
 * optee_do_call_with_ctx() - Invoke OP-TEE in secure world
 * @call_ctx:	calling context
 *
 * Does and SMC to OP-TEE in secure world and handles eventual resulting
 * Remote Procedure Calls (RPC) from OP-TEE.
 *
 * Returns return code from secure world, 0 is OK, -EAGAIN means an OCALL
 * request was received.
 */
static u32 optee_do_call_with_ctx(struct optee_call_ctx *call_ctx)
{
	struct optee *optee = tee_get_drvdata(call_ctx->ctx->teedev);
	struct optee_rpc_param param = { };
	u32 ret;

	if (call_ctx->rpc_shm) {
		param.a0 = OPTEE_SMC_CALL_RETURN_FROM_RPC;
		reg_pair_from_64(&param.a1, &param.a2,
				 (uintptr_t)call_ctx->rpc_shm);
		param.a3 = call_ctx->thread_id;
	} else {
		param.a0 = OPTEE_SMC_CALL_WITH_ARG;
		reg_pair_from_64(&param.a1, &param.a2, call_ctx->msg_parg);
	}

	while (true) {
		struct arm_smccc_res res;

		trace_optee_invoke_fn_begin(&param);
		optee->invoke_fn(param.a0, param.a1, param.a2, param.a3,
				 param.a4, param.a5, param.a6, param.a7,
				 &res);
		trace_optee_invoke_fn_end(&param, &res);

		if (res.a0 == OPTEE_SMC_RETURN_ETHREAD_LIMIT) {
			/*
			 * Out of threads in secure world, wait for a thread to
			 * become available.
			 */
			optee_cq_wait_for_completion(&optee->call_queue,
						     &call_ctx->waiter);
		} else if (OPTEE_SMC_RETURN_IS_RPC(res.a0)) {
			if (need_resched())
				cond_resched();
			param.a0 = res.a0;
			param.a1 = res.a1;
			param.a2 = res.a2;
			param.a3 = res.a3;

			if (optee_rpc_is_ocall(&param, call_ctx))
				return -EAGAIN;

			optee_handle_rpc(call_ctx->ctx, &param, call_ctx);
		} else {
			ret = res.a0;
			break;
		}
	}

	return ret;
}

/**
 * optee_do_call_with_arg() - Invoke OP-TEE in secure world
 * @ctx:	calling context
 * @parg:	physical address of message to pass to secure world
 *
 * Wraps a call to optee_do_call_with_ctx that sets up the calling context on
 * behalf of a caller that does not expect OCALLs.
 *
 * Returns return code from secure world, 0 is OK
 */
u32 optee_do_call_with_arg(struct tee_context *ctx, phys_addr_t parg)
{
	struct optee_call_ctx call_ctx = { };
	int rc;

	call_ctx.ctx = ctx;
	call_ctx.msg_parg = parg;

	call_prologue(&call_ctx);

	rc = optee_do_call_with_ctx(&call_ctx);
	if (rc == -EAGAIN) {
		pr_warn("received an unexpected OCALL, cancelling it now");
		call_ctx.rpc_arg->ret = TEEC_ERROR_NOT_SUPPORTED;
		call_ctx.rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;
		optee_do_call_with_ctx(&call_ctx);
	}

	call_epilogue(&call_ctx);

	return rc;
}

struct tee_shm *optee_get_msg_arg(struct tee_context *ctx, size_t num_params,
				  struct optee_msg_arg **msg_arg,
				  phys_addr_t *msg_parg)
{
	int rc;
	struct tee_shm *shm;
	struct optee_msg_arg *ma;

	shm = tee_shm_alloc(ctx, OPTEE_MSG_GET_ARG_SIZE(num_params),
			    TEE_SHM_MAPPED | TEE_SHM_PRIV);
	if (IS_ERR(shm))
		return shm;

	ma = tee_shm_get_va(shm, 0);
	if (IS_ERR(ma)) {
		rc = PTR_ERR(ma);
		goto out;
	}

	rc = tee_shm_get_pa(shm, 0, msg_parg);
	if (rc)
		goto out;

	memset(ma, 0, OPTEE_MSG_GET_ARG_SIZE(num_params));
	ma->num_params = num_params;
	*msg_arg = ma;
out:
	if (rc) {
		tee_shm_free(shm);
		return ERR_PTR(rc);
	}

	return shm;
}

/*
 * Requires @sem in the parent struct optee_session to be held; the caller is
 * expected to have filled in the ret and ret_origin elements of rpc_arg.
 */
static int cancel_ocall(struct optee_call_ctx *call_ctx)
{
	int rc;

	/* +2 and -2 to skip the OCALL descriptors */
	if (call_ctx->rpc_must_release) {
		process_ocall_memrefs(call_ctx->rpc_arg->params + 2,
				      call_ctx->rpc_arg->num_params - 2, false);
		call_ctx->rpc_must_release = false;
	}

	rc = optee_do_call_with_ctx(call_ctx);
	if (rc == -EAGAIN)
		pr_warn("received an OCALL while cancelling an OCALL");

	call_epilogue(call_ctx);

	return rc;
}

static int close_session(struct tee_context *ctx, u32 session)
{
	struct tee_shm *shm;
	struct optee_msg_arg *msg_arg;
	phys_addr_t msg_parg;

	shm = optee_get_msg_arg(ctx, 0, &msg_arg, &msg_parg);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	msg_arg->cmd = OPTEE_MSG_CMD_CLOSE_SESSION;
	msg_arg->session = session;
	optee_do_call_with_arg(ctx, msg_parg);

	tee_shm_free(shm);
	return 0;
}

int optee_open_session(struct tee_context *ctx,
		       struct tee_ioctl_open_session_arg *arg,
		       struct tee_param *normal_param, u32 num_normal_params,
		       struct tee_param *ocall_param)
{
	struct optee_context_data *ctxdata = ctx->data;
	struct optee_session *sess = NULL;
	struct optee_call_ctx *call_ctx = NULL;
	int sess_tmp_id;
	u64 ocall_func;
	int rc = 0;

	if (ocall_param && !ctx->cap_ocall)
		return -EOPNOTSUPP;

	ocall_func = ocall_param ? param_get_ocall_func(ocall_param) : 0;
	if (ocall_func) {
		if (arg->session > INT_MAX)
			return -EINVAL;

		sess_tmp_id = (int)arg->session;
		mutex_lock(&ctxdata->mutex);
		sess = idr_remove(&ctxdata->tmp_sess_list, sess_tmp_id);
		mutex_unlock(&ctxdata->mutex);
		if (!sess)
			return -EINVAL;

		call_ctx = &sess->call_ctx;
		if (!call_ctx->rpc_shm) {
			rc = -EINVAL;
			call_ctx->rpc_arg->ret = TEEC_ERROR_BAD_PARAMETERS;
			call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;
			goto exit_cancel;
		}

		rc = process_ocall_reply(arg->ret, arg->ret_origin,
					 normal_param, num_normal_params,
					 ocall_param, call_ctx);
		if (rc)
			goto exit_cancel;
	} else {
		sess = kzalloc(sizeof(*sess), GFP_KERNEL);
		if (!sess)
			return -ENOMEM;

		call_ctx = &sess->call_ctx;
		/* +2 for the meta parameters added below */
		call_ctx->msg_shm = optee_get_msg_arg(ctx,
						      num_normal_params + 2,
						      &call_ctx->msg_arg,
						      &call_ctx->msg_parg);
		if (IS_ERR(call_ctx->msg_shm)) {
			rc = PTR_ERR(call_ctx->msg_shm);
			goto exit_free;
		}

		call_ctx->ctx = ctx;
		call_ctx->msg_arg->cmd = OPTEE_MSG_CMD_OPEN_SESSION;
		call_ctx->msg_arg->cancel_id = arg->cancel_id;

		/*
		 * Initialize and add the meta parameters needed when opening a
		 * session.
		 */
		call_ctx->msg_arg->params[0].attr =
			OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;
		call_ctx->msg_arg->params[1].attr =
			OPTEE_MSG_ATTR_TYPE_VALUE_INPUT | OPTEE_MSG_ATTR_META;
		memcpy(&call_ctx->msg_arg->params[0].u.value, arg->uuid,
		       sizeof(arg->uuid));
		call_ctx->msg_arg->params[1].u.value.c = arg->clnt_login;
		rc = tee_session_calc_client_uuid((uuid_t *)
			&call_ctx->msg_arg->params[1].u.value,
			arg->clnt_login, arg->clnt_uuid);
		if (rc)
			goto exit_free_shm;

		rc = optee_to_msg_param(call_ctx->msg_arg->params + 2,
					num_normal_params, normal_param);
		if (rc)
			goto exit_free_shm;

		call_prologue(call_ctx);
	}

	rc = optee_do_call_with_ctx(call_ctx);
	if (rc == -EAGAIN) {
		rc = process_ocall_request(normal_param, num_normal_params,
					   ocall_param, call_ctx);
		if (rc)
			goto exit_cancel;

		/*
		 * 'sess' becomes globally visible after adding it to the IDR,
		 * so do not touch it once the mutex is unlocked.
		 */
		mutex_lock(&ctxdata->mutex);
		sess_tmp_id = idr_alloc(&ctxdata->tmp_sess_list, sess, 1, 0,
					GFP_KERNEL);
		if (sess_tmp_id >= 1)
			sess->session_id = sess_tmp_id;
		mutex_unlock(&ctxdata->mutex);
		if (sess_tmp_id < 0) {
			rc = sess_tmp_id;
			call_ctx->rpc_arg->ret = TEEC_ERROR_OUT_OF_MEMORY;
			call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;
			goto exit_cancel;
		}

		arg->session = sess_tmp_id;
	} else {
		call_epilogue(call_ctx);

		if (rc) {
			arg->ret = TEEC_ERROR_COMMUNICATION;
			arg->ret_origin = TEEC_ORIGIN_COMMS;
		} else {
			arg->ret = call_ctx->msg_arg->ret;
			arg->ret_origin = call_ctx->msg_arg->ret_origin;
		}

		if (optee_from_msg_param(normal_param, num_normal_params,
					 call_ctx->msg_arg->params + 2)) {
			if (arg->ret == TEEC_SUCCESS)
				close_session(ctx, call_ctx->msg_arg->session);

			arg->ret = TEEC_ERROR_COMMUNICATION;
			arg->ret_origin = TEEC_ORIGIN_COMMS;
		}

		if (arg->ret)
			goto exit_clear_free_all;

		/*
		 * A new session has been created, initialize it and add it to
		 * the list.
		 */
		sema_init(&sess->sem, 1);
		arg->session = call_ctx->msg_arg->session;
		sess->session_id = call_ctx->msg_arg->session;

		tee_shm_free(call_ctx->msg_shm);
		clear_call_ctx(call_ctx);

		mutex_lock(&ctxdata->mutex);
		list_add(&sess->list_node, &ctxdata->sess_list);
		mutex_unlock(&ctxdata->mutex);

		param_clear_ocall(ocall_param);
	}

	return rc;

exit_cancel:
	/* See comment in optee_cancel_open_session_ocall */
	if (cancel_ocall(call_ctx) == 0 &&
	    call_ctx->msg_arg->ret == TEEC_SUCCESS)
		close_session(ctx, call_ctx->msg_arg->session);
	optee_from_msg_param(normal_param, num_normal_params,
			     call_ctx->msg_arg->params);
exit_clear_free_all:
	param_clear_ocall(ocall_param);
exit_free_shm:
	tee_shm_free(call_ctx->msg_shm);
exit_free:
	kfree(sess);
	return rc;
}

void optee_cancel_open_session_ocall(struct optee_session *sess)
{
	struct optee_call_ctx *call_ctx = &sess->call_ctx;

	call_ctx->rpc_arg->ret = TEEC_ERROR_TARGET_DEAD;
	call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;

	/*
	 * Reaching this function means an OCALL is pending during session open
	 * but the CA has terminated abnormally. As such, the OCALL is
	 * cancelled. However, there is a chance that the TA's session open
	 * handler ignores the cancellation and lets the session open anyway. If
	 * that happens, close it.
	 */
	if (cancel_ocall(&sess->call_ctx) == 0 &&
	    call_ctx->msg_arg->ret == TEEC_SUCCESS)
		close_session(call_ctx->ctx, call_ctx->msg_arg->session);

	/*
	 * Decrease the ref count on all shared memory pointers passed into the
	 * original function invocation.
	 */
	process_ocall_memrefs(call_ctx->msg_arg->params,
			      call_ctx->msg_arg->num_params, false);

	tee_shm_free(call_ctx->msg_shm);
	kfree(sess);
}

int optee_close_session(struct tee_context *ctx, u32 session)
{
	struct optee_context_data *ctxdata = ctx->data;
	struct optee_session *sess;

	/* Check that the session is valid and remove it from the list */
	mutex_lock(&ctxdata->mutex);
	sess = find_session(ctxdata, session);
	if (sess)
		list_del(&sess->list_node);
	mutex_unlock(&ctxdata->mutex);
	if (!sess)
		return -EINVAL;

	/*
	 * If another thread found the session before we removed it from the
	 * list and that thread is operating on the session object itself, wait
	 * until it is done before we destroy it.
	 */
	down(&sess->sem);

	if (sess->call_ctx.rpc_shm)
		optee_cancel_invoke_function_ocall(&sess->call_ctx);

	kfree(sess);
	close_session(ctx, session);

	return 0;
}

int optee_invoke_func(struct tee_context *ctx, struct tee_ioctl_invoke_arg *arg,
		      struct tee_param *normal_param, u32 num_normal_params,
		      struct tee_param *ocall_param)
{
	struct optee_context_data *ctxdata = ctx->data;
	struct optee_call_ctx *call_ctx;
	struct optee_session *sess;
	u64 ocall_func;
	int rc = 0;

	if (ocall_param && !ctx->cap_ocall) {
		rc = -EOPNOTSUPP;
		goto exit;
	}

	/* Check that the session is valid */
	mutex_lock(&ctxdata->mutex);
	sess = find_session(ctxdata, arg->session);
	if (sess)
		down(&sess->sem);
	mutex_unlock(&ctxdata->mutex);
	if (!sess)
		return -EINVAL;

	call_ctx = &sess->call_ctx;
	ocall_func = ocall_param ? param_get_ocall_func(ocall_param) : 0;
	if (ocall_func) {
		/* The current call is a reply to an OCALL request */

		if (!call_ctx->rpc_shm) {
			rc = -EINVAL;
			goto exit;
		}

		rc = process_ocall_reply(arg->ret, arg->ret_origin,
					 normal_param, num_normal_params,
					 ocall_param, call_ctx);
		if (rc)
			goto exit_cancel;
	} else {
		/*
		 * The current call is an invocation that may result in an OCALL
		 * request.
		 */

		if (call_ctx->rpc_shm) {
			rc = -EINVAL;
			call_ctx->rpc_arg->ret = TEEC_ERROR_BAD_PARAMETERS;
			call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;
			goto exit_cancel;
		}

		call_ctx->msg_shm = optee_get_msg_arg(ctx, num_normal_params,
						      &call_ctx->msg_arg,
						      &call_ctx->msg_parg);
		if (IS_ERR(call_ctx->msg_shm)) {
			rc = PTR_ERR(call_ctx->msg_shm);
			goto exit_clear;
		}

		call_ctx->ctx = ctx;
		call_ctx->msg_arg->cmd = OPTEE_MSG_CMD_INVOKE_COMMAND;
		call_ctx->msg_arg->func = arg->func;
		call_ctx->msg_arg->session = arg->session;
		call_ctx->msg_arg->cancel_id = arg->cancel_id;

		rc = optee_to_msg_param(call_ctx->msg_arg->params,
					num_normal_params, normal_param);
		if (rc) {
			tee_shm_free(call_ctx->msg_shm);
			goto exit_clear;
		}

		call_prologue(call_ctx);
	}

	rc = optee_do_call_with_ctx(call_ctx);
	if (rc == -EAGAIN) {
		rc = process_ocall_request(normal_param, num_normal_params,
					   ocall_param, call_ctx);
		if (rc)
			goto exit_cancel;
	} else {
		call_epilogue(call_ctx);

		arg->ret = call_ctx->msg_arg->ret;
		arg->ret_origin = call_ctx->msg_arg->ret_origin;

		if (rc) {
			arg->ret = TEEC_ERROR_COMMUNICATION;
			arg->ret_origin = TEEC_ORIGIN_COMMS;
		}

		if (optee_from_msg_param(normal_param, num_normal_params,
					 call_ctx->msg_arg->params)) {
			arg->ret = TEEC_ERROR_COMMUNICATION;
			arg->ret_origin = TEEC_ORIGIN_COMMS;
		}

		tee_shm_free(call_ctx->msg_shm);
		clear_call_ctx(call_ctx);
		param_clear_ocall(ocall_param);
	}

	up(&sess->sem);
	return rc;

exit_cancel:
	cancel_ocall(call_ctx);
	optee_from_msg_param(normal_param, num_normal_params,
			     call_ctx->msg_arg->params);
	tee_shm_free(call_ctx->msg_shm);
	param_clear_ocall(ocall_param);
exit_clear:
	clear_call_ctx(call_ctx);
exit:
	up(&sess->sem);
	return rc;
}

/* Requires @sem in the parent struct optee_session to be held */
void optee_cancel_invoke_function_ocall(struct optee_call_ctx *call_ctx)
{
	call_ctx->rpc_arg->ret = TEEC_ERROR_TARGET_DEAD;
	call_ctx->rpc_arg->ret_origin = TEEC_ORIGIN_COMMS;

	cancel_ocall(call_ctx);

	/*
	 * Decrease the ref count on all shared memory pointers passed into the
	 * original function invocation.
	 */
	process_ocall_memrefs(call_ctx->msg_arg->params,
			      call_ctx->msg_arg->num_params, false);

	tee_shm_free(call_ctx->msg_shm);
	clear_call_ctx(call_ctx);
}

int optee_cancel_req(struct tee_context *ctx, u32 cancel_id, u32 session)
{
	struct optee_context_data *ctxdata = ctx->data;
	struct tee_shm *shm;
	struct optee_msg_arg *msg_arg;
	phys_addr_t msg_parg;
	struct optee_session *sess;

	/* Check that the session is valid */
	mutex_lock(&ctxdata->mutex);
	sess = find_session(ctxdata, session);
	mutex_unlock(&ctxdata->mutex);
	if (!sess)
		return -EINVAL;

	shm = optee_get_msg_arg(ctx, 0, &msg_arg, &msg_parg);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	msg_arg->cmd = OPTEE_MSG_CMD_CANCEL;
	msg_arg->session = session;
	msg_arg->cancel_id = cancel_id;
	optee_do_call_with_arg(ctx, msg_parg);

	tee_shm_free(shm);
	return 0;
}

/**
 * optee_enable_shm_cache() - Enables caching of some shared memory allocation
 *			      in OP-TEE
 * @optee:	main service struct
 */
void optee_enable_shm_cache(struct optee *optee)
{
	struct optee_call_waiter w;

	/* We need to retry until secure world isn't busy. */
	optee_cq_wait_init(&optee->call_queue, &w);
	while (true) {
		struct arm_smccc_res res;

		optee->invoke_fn(OPTEE_SMC_ENABLE_SHM_CACHE, 0, 0, 0, 0, 0, 0,
				 0, &res);
		if (res.a0 == OPTEE_SMC_RETURN_OK)
			break;
		optee_cq_wait_for_completion(&optee->call_queue, &w);
	}
	optee_cq_wait_final(&optee->call_queue, &w);
}

/**
 * __optee_disable_shm_cache() - Disables caching of some shared memory
 *                               allocation in OP-TEE
 * @optee:	main service struct
 * @is_mapped:	true if the cached shared memory addresses were mapped by this
 *		kernel, are safe to dereference, and should be freed
 */
static void __optee_disable_shm_cache(struct optee *optee, bool is_mapped)
{
	struct optee_call_waiter w;

	/* We need to retry until secure world isn't busy. */
	optee_cq_wait_init(&optee->call_queue, &w);
	while (true) {
		union {
			struct arm_smccc_res smccc;
			struct optee_smc_disable_shm_cache_result result;
		} res;

		optee->invoke_fn(OPTEE_SMC_DISABLE_SHM_CACHE, 0, 0, 0, 0, 0, 0,
				 0, &res.smccc);
		if (res.result.status == OPTEE_SMC_RETURN_ENOTAVAIL)
			break; /* All shm's freed */
		if (res.result.status == OPTEE_SMC_RETURN_OK) {
			struct tee_shm *shm;

			/*
			 * Shared memory references that were not mapped by
			 * this kernel must be ignored to prevent a crash.
			 */
			if (!is_mapped)
				continue;

			shm = reg_pair_to_ptr(res.result.shm_upper32,
					      res.result.shm_lower32);
			tee_shm_free(shm);
		} else {
			optee_cq_wait_for_completion(&optee->call_queue, &w);
		}
	}
	optee_cq_wait_final(&optee->call_queue, &w);
}

/**
 * optee_disable_shm_cache() - Disables caching of mapped shared memory
 *                             allocations in OP-TEE
 * @optee:	main service struct
 */
void optee_disable_shm_cache(struct optee *optee)
{
	return __optee_disable_shm_cache(optee, true);
}

/**
 * optee_disable_unmapped_shm_cache() - Disables caching of shared memory
 *                                      allocations in OP-TEE which are not
 *                                      currently mapped
 * @optee:	main service struct
 */
void optee_disable_unmapped_shm_cache(struct optee *optee)
{
	return __optee_disable_shm_cache(optee, false);
}

#define PAGELIST_ENTRIES_PER_PAGE				\
	((OPTEE_MSG_NONCONTIG_PAGE_SIZE / sizeof(u64)) - 1)

/**
 * optee_fill_pages_list() - write list of user pages to given shared
 * buffer.
 *
 * @dst: page-aligned buffer where list of pages will be stored
 * @pages: array of pages that represents shared buffer
 * @num_pages: number of entries in @pages
 * @page_offset: offset of user buffer from page start
 *
 * @dst should be big enough to hold list of user page addresses and
 *	links to the next pages of buffer
 */
void optee_fill_pages_list(u64 *dst, struct page **pages, int num_pages,
			   size_t page_offset)
{
	int n = 0;
	phys_addr_t optee_page;
	/*
	 * Refer to OPTEE_MSG_ATTR_NONCONTIG description in optee_msg.h
	 * for details.
	 */
	struct {
		u64 pages_list[PAGELIST_ENTRIES_PER_PAGE];
		u64 next_page_data;
	} *pages_data;

	/*
	 * Currently OP-TEE uses 4k page size and it does not looks
	 * like this will change in the future.  On other hand, there are
	 * no know ARM architectures with page size < 4k.
	 * Thus the next built assert looks redundant. But the following
	 * code heavily relies on this assumption, so it is better be
	 * safe than sorry.
	 */
	BUILD_BUG_ON(PAGE_SIZE < OPTEE_MSG_NONCONTIG_PAGE_SIZE);

	pages_data = (void *)dst;
	/*
	 * If linux page is bigger than 4k, and user buffer offset is
	 * larger than 4k/8k/12k/etc this will skip first 4k pages,
	 * because they bear no value data for OP-TEE.
	 */
	optee_page = page_to_phys(*pages) +
		round_down(page_offset, OPTEE_MSG_NONCONTIG_PAGE_SIZE);

	while (true) {
		pages_data->pages_list[n++] = optee_page;

		if (n == PAGELIST_ENTRIES_PER_PAGE) {
			pages_data->next_page_data =
				virt_to_phys(pages_data + 1);
			pages_data++;
			n = 0;
		}

		optee_page += OPTEE_MSG_NONCONTIG_PAGE_SIZE;
		if (!(optee_page & ~PAGE_MASK)) {
			if (!--num_pages)
				break;
			pages++;
			optee_page = page_to_phys(*pages);
		}
	}
}

/*
 * The final entry in each pagelist page is a pointer to the next
 * pagelist page.
 */
static size_t get_pages_list_size(size_t num_entries)
{
	int pages = DIV_ROUND_UP(num_entries, PAGELIST_ENTRIES_PER_PAGE);

	return pages * OPTEE_MSG_NONCONTIG_PAGE_SIZE;
}

u64 *optee_allocate_pages_list(size_t num_entries)
{
	return alloc_pages_exact(get_pages_list_size(num_entries), GFP_KERNEL);
}

void optee_free_pages_list(void *list, size_t num_entries)
{
	free_pages_exact(list, get_pages_list_size(num_entries));
}

static bool is_normal_memory(pgprot_t p)
{
#if defined(CONFIG_ARM)
	return (((pgprot_val(p) & L_PTE_MT_MASK) == L_PTE_MT_WRITEALLOC) ||
		((pgprot_val(p) & L_PTE_MT_MASK) == L_PTE_MT_WRITEBACK));
#elif defined(CONFIG_ARM64)
	return (pgprot_val(p) & PTE_ATTRINDX_MASK) == PTE_ATTRINDX(MT_NORMAL);
#else
#error "Unuspported architecture"
#endif
}

static int __check_mem_type(struct vm_area_struct *vma, unsigned long end)
{
	while (vma && is_normal_memory(vma->vm_page_prot)) {
		if (vma->vm_end >= end)
			return 0;
		vma = vma->vm_next;
	}

	return -EINVAL;
}

static int check_mem_type(unsigned long start, size_t num_pages)
{
	struct mm_struct *mm = current->mm;
	int rc;

	/*
	 * Allow kernel address to register with OP-TEE as kernel
	 * pages are configured as normal memory only.
	 */
	if (virt_addr_valid(start))
		return 0;

	mmap_read_lock(mm);
	rc = __check_mem_type(find_vma(mm, start),
			      start + num_pages * PAGE_SIZE);
	mmap_read_unlock(mm);

	return rc;
}

int optee_shm_register(struct tee_context *ctx, struct tee_shm *shm,
		       struct page **pages, size_t num_pages,
		       unsigned long start)
{
	struct tee_shm *shm_arg = NULL;
	struct optee_msg_arg *msg_arg;
	u64 *pages_list;
	phys_addr_t msg_parg;
	int rc;

	if (!num_pages)
		return -EINVAL;

	rc = check_mem_type(start, num_pages);
	if (rc)
		return rc;

	pages_list = optee_allocate_pages_list(num_pages);
	if (!pages_list)
		return -ENOMEM;

	shm_arg = optee_get_msg_arg(ctx, 1, &msg_arg, &msg_parg);
	if (IS_ERR(shm_arg)) {
		rc = PTR_ERR(shm_arg);
		goto out;
	}

	optee_fill_pages_list(pages_list, pages, num_pages,
			      tee_shm_get_page_offset(shm));

	msg_arg->cmd = OPTEE_MSG_CMD_REGISTER_SHM;
	msg_arg->params->attr = OPTEE_MSG_ATTR_TYPE_TMEM_OUTPUT |
				OPTEE_MSG_ATTR_NONCONTIG;
	msg_arg->params->u.tmem.shm_ref = (unsigned long)shm;
	msg_arg->params->u.tmem.size = tee_shm_get_size(shm);
	/*
	 * In the least bits of msg_arg->params->u.tmem.buf_ptr we
	 * store buffer offset from 4k page, as described in OP-TEE ABI.
	 */
	msg_arg->params->u.tmem.buf_ptr = virt_to_phys(pages_list) |
	  (tee_shm_get_page_offset(shm) & (OPTEE_MSG_NONCONTIG_PAGE_SIZE - 1));

	if (optee_do_call_with_arg(ctx, msg_parg) ||
	    msg_arg->ret != TEEC_SUCCESS)
		rc = -EINVAL;

	tee_shm_free(shm_arg);
out:
	optee_free_pages_list(pages_list, num_pages);
	return rc;
}

int optee_shm_unregister(struct tee_context *ctx, struct tee_shm *shm)
{
	struct tee_shm *shm_arg;
	struct optee_msg_arg *msg_arg;
	phys_addr_t msg_parg;
	int rc = 0;

	shm_arg = optee_get_msg_arg(ctx, 1, &msg_arg, &msg_parg);
	if (IS_ERR(shm_arg))
		return PTR_ERR(shm_arg);

	msg_arg->cmd = OPTEE_MSG_CMD_UNREGISTER_SHM;

	msg_arg->params[0].attr = OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
	msg_arg->params[0].u.rmem.shm_ref = (unsigned long)shm;

	if (optee_do_call_with_arg(ctx, msg_parg) ||
	    msg_arg->ret != TEEC_SUCCESS)
		rc = -EINVAL;
	tee_shm_free(shm_arg);
	return rc;
}

int optee_shm_register_supp(struct tee_context *ctx, struct tee_shm *shm,
			    struct page **pages, size_t num_pages,
			    unsigned long start)
{
	/*
	 * We don't want to register supplicant memory in OP-TEE.
	 * Instead information about it will be passed in RPC code.
	 */
	return check_mem_type(start, num_pages);
}

int optee_shm_unregister_supp(struct tee_context *ctx, struct tee_shm *shm)
{
	return 0;
}
