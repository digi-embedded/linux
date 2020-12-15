========================================
STM32 DDR Performance Monitor (DDRPERFM)
========================================

The DDRPERFM is the DDR Performance Monitor embedded in STM32MP1 SOC.
See Documentation/arm/stm32/stm32mp157-overview.rst to get access to
STM32MP157 reference manual RM0436 where DDRPERFM is described.


The five following counters are supported by stm32-ddr-pmu driver:
	cnt0: read operations counters		(read_cnt)
	cnt1: write operations counters		(write_cnt)
	cnt2: active state counters		(activate_cnt)
	cnt3: idle state counters		(idle_cnt)
	tcnt: time count, present for all sets	(time_cnt)

The stm32-ddr-pmu driver relies on the perf PMU framework to expose the
counters via sysfs:

    .. code-block:: bash

	$ ls /sys/bus/event_source/devices/ddrperfm/events
	activate_cnt  idle_cnt  read_cnt  time_cnt  write_cnt


The perf PMU framework is usually invoked via the 'perf stat' tool.

The DDRPERFM is a system monitor that cannot isolate the traffic coming from a
given thread or CPU, that is why stm32-ddr-pmu driver rejects any 'perf stat'
call that does not request a system-wide collection: the '-a, --all-cpus'
option is mandatory!

Example:

    .. code-block:: bash

        $ perf stat -e ddrperfm/read_cnt/,ddrperfm/time_cnt/ -a sleep 20
	Performance counter stats for 'system wide':

	         342541560      ddrperfm/read_cnt/
	       10660011400      ddrperfm/time_cnt/

	      20.021068551 seconds time elapsed

