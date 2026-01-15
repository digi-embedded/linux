ccimx8m_som_overlays := ccimx8m_bt.dtbo \
			ccimx8m_mca-keypad.dtbo \
			ccimx8m_wifi.dtbo

ccimx8m_dvk_overlays := ccimx8m-dvk_flexspi.dtbo \
			ccimx8m-dvk_gpio-watchdog.dtbo \
			ccimx8m-dvk_hsd101pfw2-lvds.dtbo \
			ccimx8m-dvk_lvds.dtbo \
			ccimx8m-dvk_user-leds.dtbo

ccimx8x_som_overlays := ccimx8x_bt.dtbo \
			ccimx8x_cm4.dtbo \
			ccimx8x_mca-keypad.dtbo \
			ccimx8x_quad.dtbo \
			ccimx8x_wifi.dtbo

ccimx8x_dvk_overlays := ccimx8x-sbc-pro_flexcan1.dtbo \
			ccimx8x-sbc-pro_gpio-watchdog.dtbo \
			ccimx8x-sbc-pro_lpuart3.dtbo \
			ccimx8x-sbc-pro_flexspi.dtbo \
			ccimx8x-sbc-pro_hsd101pfw2-lvds.dtbo \
			ccimx8x-sbc-pro_lt8912-hdmi-dsi0.dtbo \
			ccimx8x-sbc-pro_lvds1.dtbo \
			ccimx8x-sbc-pro_parallel-camera.dtbo \
			ccimx8x-sbc-pro_pcie-card.dtbo \
			ccimx8x-sbc-pro_pcie-modem.dtbo \
			ccimx8x-sbc-pro_user-leds.dtbo \
			ccimx8x-sbc-pro_v1-v3.dtbo

ccimx9_som_overlays := ccimx9_bt.dtbo \
			ccimx9_bt-dtm.dtbo \
			ccimx9_wifi.dtbo

ccimx95_som_overlays := ccimx95_bt.dtbo \
			ccimx9_wifi.dtbo

ccimx9_dvk_overlays := ccimx9-dvk_enet2.dtbo \
			ccimx9-dvk_eqos-tsn.dtbo

ccimx91_dvk_overlays := ccimx9-dvk_mikroe-accel2-click.dtbo \
			ccimx9-dvk_mikroe-gyro-click.dtbo

ccimx93_som_overlays := ccimx93_npu.dtbo \
			ccimx93_single-core-512M.dtbo

ccimx93_dvk_overlays := ccimx9-dvk_mikroe-accel2-click.dtbo \
			ccimx9-dvk_mikroe-gyro-click.dtbo \
			ccimx93_emulate-ccimx91.dtbo \
			ccimx93-dvk_dlc0200ccp04df-mipi-dsi.dtbo \
			ccimx93-dvk_lvds.dtbo \
			ccimx93-dvk_nhd-3-5-640480ef-msxp-mipi-dsi.dtbo

ccimx8mm-dtbs :=	ccimx8mm-dvk.dtb \
			$(ccimx8m_dvk_overlays) \
			$(ccimx8m_som_overlays) \
			ccimx8mm_cm4.dtbo

ccimx8mn-dtbs :=	ccimx8mn-dvk.dtb \
			$(ccimx8m_dvk_overlays) \
			$(ccimx8m_som_overlays) \
			ccimx8mn_cm7.dtbo \
			ccimx8mn-dvk_v1-v2.dtbo \
			ccimx8mn_v1.dtbo

ccimx8xsbcpro-dtbs := 	ccimx8x-sbc-pro.dtb \
			$(ccimx8x_dvk_overlays) \
			$(ccimx8x_som_overlays)

ccimx8xmfg-dtbs :=	ccimx8x-mfg-functional.dtb \
			ccimx8x_mfg-functional.dtbo

ccimx91-dtbs := 	ccimx91-dvk.dtb \
			$(ccimx91_dvk_overlays) \
			$(ccimx9_dvk_overlays) \
			$(ccimx9_som_overlays)

ccimx93-dtbs := 	ccimx93-dvk.dtb \
			$(ccimx93_dvk_overlays) \
			$(ccimx93_som_overlays) \
			$(ccimx9_dvk_overlays) \
			$(ccimx9_som_overlays)

ccimx95_dvk_overlays := ccimx95-dvk_enet2.dtbo \
			ccimx95-dvk_lvds0_g101evn010.dtbo \
			ccimx95-dvk_lvds1_g101evn010.dtbo \
			ccimx95-dvk_mikroe-accel2-click.dtbo \
			ccimx95-dvk_mikroe-gyro-click.dtbo \
			ccimx95-dvk_pcam5c.dtbo

ccimx95-dtbs := 	ccimx95-dvk.dtb \
			$(ccimx95_som_overlays) \
			$(ccimx95_dvk_overlays)

dtb-$(CONFIG_ARCH_MXC) += ccimx8mm.dtb \
			  ccimx8mm-mfg-functional.dtb \
			  ccimx8mm-mfg-functional-wb.dtb \
			  ccimx8mn.dtb \
			  ccimx8mn-mfg-functional.dtb \
			  ccimx8mn-mfg-functional-wb.dtb \
			  ccimx8xsbcpro.dtb \
			  ccimx8xmfg.dtb \
			  ccimx8x-mfg-interim.dtb \
			  ccimx8x-mfg-functional-wb.dtb \
			  ccimx8x-sbc-express.dtb \
			  ccimx8x-sbc-express-mfg.dtb \
			  ccimx8x-sbc-pro-mfg.dtb \
			  ccimx91.dtb \
			  ccimx91-mfg-functional.dtb \
			  ccimx91-mfg-functional-wb.dtb \
			  ccimx93.dtb \
			  ccimx93-mfg-functional.dtb \
			  ccimx93-mfg-functional-wb.dtb \
			  ccimx95.dtb
