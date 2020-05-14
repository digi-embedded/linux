#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/mfd/abx500/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/abx500/ab8500-sysctrl.h>
#include <linux/mfd/abx500/ab8500-codec.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "i2s-codec.h"

#define CLK_32K_OUT2_DISABLE 0x01
#define INACTIVE_RESET_AUDIO 0x02
#define ENABLE_AUDIO_CLK_TO_AUDIO_BLK 0x10
#define ENABLE_VINTCORE12_SUPPLY 0x04
#define GPIO27_DIR_OUTPUT 0x04
#define GPIO29_DIR_OUTPUT 0x10
#define GPIO31_DIR_OUTPUT 0x40


#define AB8500_GPIO_DIR4_REG 0x13


#define AB8500_NR_OF_ANC_COEFF_BANKS 2



#define AB8500_ANC_SM_DELAY 2000
struct filter_control {
 long min, max;
 unsigned int count;
 long value[128];
};


static const char * const enum_sid_state[] = {
 "Unconfigured",
 "Apply FIR",
 "FIR is configured",
};
enum sid_state {
 SID_UNCONFIGURED = 0,
 SID_APPLY_FIR = 1,
 SID_FIR_CONFIGURED = 2,
};

static const char * const enum_anc_state[] = {
 "Unconfigured",
 "Apply FIR and IIR",
 "FIR and IIR are configured",
 "Apply FIR",
 "FIR is configured",
 "Apply IIR",
 "IIR is configured"
};
enum anc_state {
 ANC_UNCONFIGURED = 0,
 ANC_APPLY_FIR_IIR = 1,
 ANC_FIR_IIR_CONFIGURED = 2,
 ANC_APPLY_FIR = 3,
 ANC_FIR_CONFIGURED = 4,
 ANC_APPLY_IIR = 5,
 ANC_IIR_CONFIGURED = 6
};


enum amic_idx {
 AMIC_IDX_1A,
 AMIC_IDX_1B,
 AMIC_IDX_2
};

struct ab8500_codec_drvdata_dbg {
 struct regulator *vaud;
 struct regulator *vamic1;
 struct regulator *vamic2;
 struct regulator *vdmic;
};


struct ab8500_codec_drvdata {
 struct regmap *regmap;
 struct mutex ctrl_lock;


 long *sid_fir_values;
 enum sid_state sid_status;


 long *anc_fir_values;
 long *anc_iir_values;
 enum anc_state anc_status;
};

static inline const char *amic_micbias_str(enum amic_micbias micbias)
{
 switch (micbias) {
 case AMIC_MICBIAS_VAMIC1:
  return "VAMIC1";
 case AMIC_MICBIAS_VAMIC2:
  return "VAMIC2";
 default:
  return "Unknown";
 }
}

static inline const char *amic_type_str(enum amic_type type)
{
 switch (type) {
 case AMIC_TYPE_DIFFERENTIAL:
  return "DIFFERENTIAL";
 case AMIC_TYPE_SINGLE_ENDED:
  return "SINGLE ENDED";
 default:
  return "Unknown";
 }
}






static int ab8500_codec_read_reg(void *context, unsigned int reg,
     unsigned int *value)
{
 *value = 0;
 return 0;
}


static int ab8500_codec_write_reg(void *context, unsigned int reg,
      unsigned int value)
{





 return 0;
}

static const struct regmap_config ab8500_codec_regmap = {
 .reg_read = ab8500_codec_read_reg,
 .reg_write = ab8500_codec_write_reg,
};
static const char * const enum_ear_lineout_source[] = {"Headset Left",
      "Speaker Left"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ear_lineout_source, AB8500_DMICFILTCONF,
   AB8500_DMICFILTCONF_DA3TOEAR, enum_ear_lineout_source);
static const struct snd_kcontrol_new dapm_ear_lineout_source =
 SOC_DAPM_ENUM("Earpiece or LineOut Mono Source",
  dapm_enum_ear_lineout_source);




static const char * const enum_lineout_source[] = {"Mono Path", "Stereo Path"};
static SOC_ENUM_DOUBLE_DECL(dapm_enum_lineout_source, AB8500_ANACONF5,
   AB8500_ANACONF5_HSLDACTOLOL,
   AB8500_ANACONF5_HSRDACTOLOR, enum_lineout_source);
static const struct snd_kcontrol_new dapm_lineout_source[] = {
 SOC_DAPM_ENUM("LineOut Source", dapm_enum_lineout_source),
};




static const char * const enum_HFx_sel[] = {"Audio Path", "ANC"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_HFl_sel, AB8500_DIGMULTCONF2,
   AB8500_DIGMULTCONF2_HFLSEL, enum_HFx_sel);
static const struct snd_kcontrol_new dapm_HFl_select[] = {
 SOC_DAPM_ENUM("Speaker Left Source", dapm_enum_HFl_sel),
};


static SOC_ENUM_SINGLE_DECL(dapm_enum_HFr_sel, AB8500_DIGMULTCONF2,
   AB8500_DIGMULTCONF2_HFRSEL, enum_HFx_sel);
static const struct snd_kcontrol_new dapm_HFr_select[] = {
 SOC_DAPM_ENUM("Speaker Right Source", dapm_enum_HFr_sel),
};




static const char * const enum_mic1ab_sel[] = {"Mic 1b", "Mic 1a"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_mic1ab_sel, AB8500_ANACONF3,
   AB8500_ANACONF3_MIC1SEL, enum_mic1ab_sel);
static const struct snd_kcontrol_new dapm_mic1ab_mux[] = {
 SOC_DAPM_ENUM("Mic 1a or 1b Select", dapm_enum_mic1ab_sel),
};


static const char * const enum_ad3_sel[] = {"Mic 1", "DMic 3"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad3_sel, AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_AD3SEL, enum_ad3_sel);
static const struct snd_kcontrol_new dapm_ad3_select[] = {
 SOC_DAPM_ENUM("AD3 Source Select", dapm_enum_ad3_sel),
};


static const char * const enum_ad6_sel[] = {"Mic 1", "DMic 6"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad6_sel, AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_AD6SEL, enum_ad6_sel);
static const struct snd_kcontrol_new dapm_ad6_select[] = {
 SOC_DAPM_ENUM("AD6 Source Select", dapm_enum_ad6_sel),
};




static const char * const enum_ad5_sel[] = {"Mic 2", "DMic 5"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad5_sel, AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_AD5SEL, enum_ad5_sel);
static const struct snd_kcontrol_new dapm_ad5_select[] = {
 SOC_DAPM_ENUM("AD5 Source Select", dapm_enum_ad5_sel),
};




static const char * const enum_ad1_sel[] = {"LineIn Left", "DMic 1"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad1_sel, AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_AD1SEL, enum_ad1_sel);
static const struct snd_kcontrol_new dapm_ad1_select[] = {
 SOC_DAPM_ENUM("AD1 Source Select", dapm_enum_ad1_sel),
};


static const char * const enum_mic2lr_sel[] = {"Mic 2", "LineIn Right"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_mic2lr_sel, AB8500_ANACONF3,
   AB8500_ANACONF3_LINRSEL, enum_mic2lr_sel);
static const struct snd_kcontrol_new dapm_mic2lr_select[] = {
 SOC_DAPM_ENUM("Mic 2 or LINR Select", dapm_enum_mic2lr_sel),
};


static const char * const enum_ad2_sel[] = {"LineIn Right", "DMic 2"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_ad2_sel, AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_AD2SEL, enum_ad2_sel);
static const struct snd_kcontrol_new dapm_ad2_select[] = {
 SOC_DAPM_ENUM("AD2 Source Select", dapm_enum_ad2_sel),
};




static const char * const enum_anc_in_sel[] = {"Mic 1 / DMic 6",
     "Mic 2 / DMic 5"};
static SOC_ENUM_SINGLE_DECL(dapm_enum_anc_in_sel, AB8500_DMICFILTCONF,
   AB8500_DMICFILTCONF_ANCINSEL, enum_anc_in_sel);
static const struct snd_kcontrol_new dapm_anc_in_select[] = {
 SOC_DAPM_ENUM("ANC Source", dapm_enum_anc_in_sel),
};


static const struct snd_kcontrol_new dapm_anc_enable[] = {
 SOC_DAPM_SINGLE("Switch", AB8500_ANCCONF1,
   AB8500_ANCCONF1_ENANC, 0, 0),
};


static const struct snd_kcontrol_new dapm_anc_ear_mute[] = {
 SOC_DAPM_SINGLE("Switch", AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_ANCSEL, 1, 0),
};






static const char * const enum_stfir1_in_sel[] = {
 "LineIn Left", "LineIn Right", "Mic 1", "Headset Left"
};
static SOC_ENUM_SINGLE_DECL(dapm_enum_stfir1_in_sel, AB8500_DIGMULTCONF2,
   AB8500_DIGMULTCONF2_FIRSID1SEL, enum_stfir1_in_sel);
static const struct snd_kcontrol_new dapm_stfir1_in_select[] = {
 SOC_DAPM_ENUM("Sidetone Left Source", dapm_enum_stfir1_in_sel),
};




static const char * const enum_stfir2_in_sel[] = {
 "LineIn Right", "Mic 1", "DMic 4", "Headset Right"
};
static SOC_ENUM_SINGLE_DECL(dapm_enum_stfir2_in_sel, AB8500_DIGMULTCONF2,
   AB8500_DIGMULTCONF2_FIRSID2SEL, enum_stfir2_in_sel);
static const struct snd_kcontrol_new dapm_stfir2_in_select[] = {
 SOC_DAPM_ENUM("Sidetone Right Source", dapm_enum_stfir2_in_sel),
};



static const char * const enum_pwm2vibx[] = {"Audio Path", "PWM Generator"};

static SOC_ENUM_SINGLE_DECL(dapm_enum_pwm2vib1, AB8500_PWMGENCONF1,
   AB8500_PWMGENCONF1_PWMTOVIB1, enum_pwm2vibx);

static const struct snd_kcontrol_new dapm_pwm2vib1[] = {
 SOC_DAPM_ENUM("Vibra 1 Controller", dapm_enum_pwm2vib1),
};

static SOC_ENUM_SINGLE_DECL(dapm_enum_pwm2vib2, AB8500_PWMGENCONF1,
   AB8500_PWMGENCONF1_PWMTOVIB2, enum_pwm2vibx);

static const struct snd_kcontrol_new dapm_pwm2vib2[] = {
 SOC_DAPM_ENUM("Vibra 2 Controller", dapm_enum_pwm2vib2),
};





static const struct snd_soc_dapm_widget ab8500_dapm_widgets[] = {
};




static const struct snd_soc_dapm_route ab8500_dapm_routes[] = {
};

static const struct snd_soc_dapm_route ab8500_dapm_routes_mic1a_vamicx[] = {
 {"MIC1A V-AMICx Enable", NULL, "V-AMIC1"},
 {"MIC1A V-AMICx Enable", NULL, "V-AMIC2"},
};

static const struct snd_soc_dapm_route ab8500_dapm_routes_mic1b_vamicx[] = {
 {"MIC1B V-AMICx Enable", NULL, "V-AMIC1"},
 {"MIC1B V-AMICx Enable", NULL, "V-AMIC2"},
};

static const struct snd_soc_dapm_route ab8500_dapm_routes_mic2_vamicx[] = {
 {"MIC2 V-AMICx Enable", NULL, "V-AMIC1"},
 {"MIC2 V-AMICx Enable", NULL, "V-AMIC2"},
};
static DECLARE_TLV_DB_SCALE(adx_dig_gain_tlv, -3200, 100, 1);


static DECLARE_TLV_DB_SCALE(dax_dig_gain_tlv, -6300, 100, 1);


static DECLARE_TLV_DB_SCALE(hs_ear_dig_gain_tlv, -100, 100, 1);


static const unsigned int hs_gain_tlv[] = {
 TLV_DB_RANGE_HEAD(2),
 0, 3, TLV_DB_SCALE_ITEM(-3200, 400, 0),
 4, 15, TLV_DB_SCALE_ITEM(-1800, 200, 0),
};

static DECLARE_TLV_DB_SCALE(mic_gain_tlv, 0, 100, 0);

static DECLARE_TLV_DB_SCALE(lin_gain_tlv, -1000, 200, 0);

static DECLARE_TLV_DB_SCALE(lin2hs_gain_tlv, -3800, 200, 1);


static const char * const enum_hsfadspeed[] = {"2ms", "0.5ms", "10.6ms",
     "5ms"};
static SOC_ENUM_SINGLE_DECL(soc_enum_hsfadspeed,
 AB8500_DIGMICCONF, AB8500_DIGMICCONF_HSFADSPEED, enum_hsfadspeed);

static const char * const enum_envdetthre[] = {
 "250mV", "300mV", "350mV", "400mV",
 "450mV", "500mV", "550mV", "600mV",
 "650mV", "700mV", "750mV", "800mV",
 "850mV", "900mV", "950mV", "1.00V" };
static SOC_ENUM_SINGLE_DECL(soc_enum_envdeththre,
 AB8500_ENVCPCONF, AB8500_ENVCPCONF_ENVDETHTHRE, enum_envdetthre);
static SOC_ENUM_SINGLE_DECL(soc_enum_envdetlthre,
 AB8500_ENVCPCONF, AB8500_ENVCPCONF_ENVDETLTHRE, enum_envdetthre);
static const char * const enum_envdettime[] = {
 "26.6us", "53.2us", "106us", "213us",
 "426us", "851us", "1.70ms", "3.40ms",
 "6.81ms", "13.6ms", "27.2ms", "54.5ms",
 "109ms", "218ms", "436ms", "872ms" };
static SOC_ENUM_SINGLE_DECL(soc_enum_envdettime,
 AB8500_SIGENVCONF, AB8500_SIGENVCONF_ENVDETTIME, enum_envdettime);

static const char * const enum_sinc31[] = {"Sinc 3", "Sinc 1"};
static SOC_ENUM_SINGLE_DECL(soc_enum_hsesinc, AB8500_HSLEARDIGGAIN,
   AB8500_HSLEARDIGGAIN_HSSINC1, enum_sinc31);

static const char * const enum_fadespeed[] = {"1ms", "4ms", "8ms", "16ms"};
static SOC_ENUM_SINGLE_DECL(soc_enum_fadespeed, AB8500_HSRDIGGAIN,
   AB8500_HSRDIGGAIN_FADESPEED, enum_fadespeed);



static const char * const enum_lowpow[] = {"Normal", "Low Power"};
static SOC_ENUM_SINGLE_DECL(soc_enum_eardaclowpow, AB8500_ANACONF1,
   AB8500_ANACONF1_EARDACLOWPOW, enum_lowpow);
static SOC_ENUM_SINGLE_DECL(soc_enum_eardrvlowpow, AB8500_ANACONF1,
   AB8500_ANACONF1_EARDRVLOWPOW, enum_lowpow);

static const char * const enum_av_mode[] = {"Audio", "Voice"};
static SOC_ENUM_DOUBLE_DECL(soc_enum_ad12voice, AB8500_ADFILTCONF,
 AB8500_ADFILTCONF_AD1VOICE, AB8500_ADFILTCONF_AD2VOICE, enum_av_mode);
static SOC_ENUM_DOUBLE_DECL(soc_enum_ad34voice, AB8500_ADFILTCONF,
 AB8500_ADFILTCONF_AD3VOICE, AB8500_ADFILTCONF_AD4VOICE, enum_av_mode);



static SOC_ENUM_SINGLE_DECL(soc_enum_da12voice,
   AB8500_DASLOTCONF1, AB8500_DASLOTCONF1_DA12VOICE,
   enum_av_mode);
static SOC_ENUM_SINGLE_DECL(soc_enum_da34voice,
   AB8500_DASLOTCONF3, AB8500_DASLOTCONF3_DA34VOICE,
   enum_av_mode);
static SOC_ENUM_SINGLE_DECL(soc_enum_da56voice,
   AB8500_DASLOTCONF5, AB8500_DASLOTCONF5_DA56VOICE,
   enum_av_mode);

static const char * const enum_da2hslr[] = {"Sidetone", "Audio Path"};
static SOC_ENUM_DOUBLE_DECL(soc_enum_da2hslr, AB8500_DIGMULTCONF1,
   AB8500_DIGMULTCONF1_DATOHSLEN,
   AB8500_DIGMULTCONF1_DATOHSREN, enum_da2hslr);

static const char * const enum_sinc53[] = {"Sinc 5", "Sinc 3"};
static SOC_ENUM_DOUBLE_DECL(soc_enum_dmic12sinc, AB8500_DMICFILTCONF,
   AB8500_DMICFILTCONF_DMIC1SINC3,
   AB8500_DMICFILTCONF_DMIC2SINC3, enum_sinc53);
static SOC_ENUM_DOUBLE_DECL(soc_enum_dmic34sinc, AB8500_DMICFILTCONF,
   AB8500_DMICFILTCONF_DMIC3SINC3,
   AB8500_DMICFILTCONF_DMIC4SINC3, enum_sinc53);
static SOC_ENUM_DOUBLE_DECL(soc_enum_dmic56sinc, AB8500_DMICFILTCONF,
   AB8500_DMICFILTCONF_DMIC5SINC3,
   AB8500_DMICFILTCONF_DMIC6SINC3, enum_sinc53);


static const char * const enum_da_from_slot_map[] = {"SLOT0",
     "SLOT1",
     "SLOT2",
     "SLOT3",
     "SLOT4",
     "SLOT5",
     "SLOT6",
     "SLOT7",
     "SLOT8",
     "SLOT9",
     "SLOT10",
     "SLOT11",
     "SLOT12",
     "SLOT13",
     "SLOT14",
     "SLOT15",
     "SLOT16",
     "SLOT17",
     "SLOT18",
     "SLOT19",
     "SLOT20",
     "SLOT21",
     "SLOT22",
     "SLOT23",
     "SLOT24",
     "SLOT25",
     "SLOT26",
     "SLOT27",
     "SLOT28",
     "SLOT29",
     "SLOT30",
     "SLOT31"};
static SOC_ENUM_SINGLE_DECL(soc_enum_da1slotmap,
   AB8500_DASLOTCONF1, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da2slotmap,
   AB8500_DASLOTCONF2, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da3slotmap,
   AB8500_DASLOTCONF3, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da4slotmap,
   AB8500_DASLOTCONF4, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da5slotmap,
   AB8500_DASLOTCONF5, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da6slotmap,
   AB8500_DASLOTCONF6, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da7slotmap,
   AB8500_DASLOTCONF7, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_da8slotmap,
   AB8500_DASLOTCONF8, AB8500_DASLOTCONFX_SLTODAX_SHIFT,
   enum_da_from_slot_map);


static const char * const enum_ad_to_slot_map[] = {"AD_OUT1",
     "AD_OUT2",
     "AD_OUT3",
     "AD_OUT4",
     "AD_OUT5",
     "AD_OUT6",
     "AD_OUT7",
     "AD_OUT8",
     "zeroes",
     "zeroes",
     "zeroes",
     "zeroes",
     "tristate",
     "tristate",
     "tristate",
     "tristate"};
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot0map,
   AB8500_ADSLOTSEL1, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot1map,
   AB8500_ADSLOTSEL1, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot2map,
   AB8500_ADSLOTSEL2, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot3map,
   AB8500_ADSLOTSEL2, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot4map,
   AB8500_ADSLOTSEL3, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot5map,
   AB8500_ADSLOTSEL3, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot6map,
   AB8500_ADSLOTSEL4, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot7map,
   AB8500_ADSLOTSEL4, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot8map,
   AB8500_ADSLOTSEL5, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot9map,
   AB8500_ADSLOTSEL5, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot10map,
   AB8500_ADSLOTSEL6, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot11map,
   AB8500_ADSLOTSEL6, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot12map,
   AB8500_ADSLOTSEL7, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot13map,
   AB8500_ADSLOTSEL7, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot14map,
   AB8500_ADSLOTSEL8, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot15map,
   AB8500_ADSLOTSEL8, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot16map,
   AB8500_ADSLOTSEL9, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot17map,
   AB8500_ADSLOTSEL9, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot18map,
   AB8500_ADSLOTSEL10, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot19map,
   AB8500_ADSLOTSEL10, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot20map,
   AB8500_ADSLOTSEL11, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot21map,
   AB8500_ADSLOTSEL11, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot22map,
   AB8500_ADSLOTSEL12, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot23map,
   AB8500_ADSLOTSEL12, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot24map,
   AB8500_ADSLOTSEL13, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot25map,
   AB8500_ADSLOTSEL13, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot26map,
   AB8500_ADSLOTSEL14, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot27map,
   AB8500_ADSLOTSEL14, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot28map,
   AB8500_ADSLOTSEL15, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot29map,
   AB8500_ADSLOTSEL15, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot30map,
   AB8500_ADSLOTSEL16, AB8500_ADSLOTSELX_EVEN_SHIFT,
   enum_ad_to_slot_map);
static SOC_ENUM_SINGLE_DECL(soc_enum_adslot31map,
   AB8500_ADSLOTSEL16, AB8500_ADSLOTSELX_ODD_SHIFT,
   enum_ad_to_slot_map);


static const char * const enum_mask[] = {"Unmasked", "Masked"};
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifomask,
   AB8500_FIFOCONF1, AB8500_FIFOCONF1_BFIFOMASK,
   enum_mask);
static const char * const enum_bitclk0[] = {"19_2_MHz", "38_4_MHz"};
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifo19m2,
   AB8500_FIFOCONF1, AB8500_FIFOCONF1_BFIFO19M2,
   enum_bitclk0);
static const char * const enum_slavemaster[] = {"Slave", "Master"};
static SOC_ENUM_SINGLE_DECL(soc_enum_bfifomast,
   AB8500_FIFOCONF3, AB8500_FIFOCONF3_BFIFOMAST_SHIFT,
   enum_slavemaster);


static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_sidstate, enum_sid_state);


static SOC_ENUM_SINGLE_EXT_DECL(soc_enum_ancstate, enum_anc_state);

static struct snd_kcontrol_new ab8500_ctrls[] = {
};
static int ab8500_codec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{


 struct snd_soc_codec *codec = dai->codec;
 int status;

 dev_dbg(codec->dev, "%s: Enter (fmt = 0x%x)\n", __func__, fmt);
 return 0;
}

static int ab8500_codec_set_dai_tdm_slot(struct snd_soc_dai *dai,
  unsigned int tx_mask, unsigned int rx_mask,
  int slots, int slot_width)
{
 return 0;
}

static const struct snd_soc_dai_ops ab8500_codec_ops = {
 .set_fmt = ab8500_codec_set_dai_fmt,
 .set_tdm_slot = ab8500_codec_set_dai_tdm_slot,
};

static struct snd_soc_dai_driver ab8500_codec_dai[] = {
 {
  .name = "i2s-codec-dai.0",
  .id = 0,
  .playback = {
   .stream_name = "playback",
   .channels_min = 1,
   .channels_max = 8,
   .rates = AB8500_SUPPORTED_RATE,
   .formats = AB8500_SUPPORTED_FMT,
  },
  .capture = {
   .stream_name = "capture1",
   .channels_min = 1,
   .channels_max = 8,
   .rates = AB8500_SUPPORTED_RATE,
   .formats = AB8500_SUPPORTED_FMT,
  },
  .ops = &ab8500_codec_ops,
  .symmetric_rates = 1
 },
 {
  .name = "i2s-codec-dai.1",
  .id = 1,
  .capture = {
   .stream_name = "capture2",
   .channels_min = 1,
   .channels_max = 8,
   .rates = AB8500_SUPPORTED_RATE,
   .formats = AB8500_SUPPORTED_FMT,
  },
  .ops = &ab8500_codec_ops,
  .symmetric_rates = 1
 }
};
static int ab8500_codec_probe(struct snd_soc_codec *codec)
{
 int status = 0;
 return status;
}

static struct snd_soc_codec_driver ab8500_codec_driver = {

	.probe =		ab8500_codec_probe,
	.component_driver = {
		.controls =		ab8500_ctrls,
		.num_controls =		ARRAY_SIZE(ab8500_ctrls),
		.dapm_widgets =		ab8500_dapm_widgets,
		.num_dapm_widgets =	ARRAY_SIZE(ab8500_dapm_widgets),
		.dapm_routes =		ab8500_dapm_routes,
		.num_dapm_routes =	ARRAY_SIZE(ab8500_dapm_routes),
	},
};

static int ab8500_codec_driver_probe(struct platform_device *pdev)
{
 int status, err;
 struct ab8500_codec_drvdata *drvdata;
 struct clk *codec_clk;

 dev_dbg(&pdev->dev, "%s: Enter.\n", __func__);


 drvdata = devm_kzalloc(&pdev->dev, sizeof(struct ab8500_codec_drvdata),
   GFP_KERNEL);
 if (!drvdata)
  return -ENOMEM;
 drvdata->sid_status = SID_UNCONFIGURED;
 drvdata->anc_status = ANC_UNCONFIGURED;
 dev_set_drvdata(&pdev->dev, drvdata);

 drvdata->regmap = devm_regmap_init(&pdev->dev, NULL, &pdev->dev,
        &ab8500_codec_regmap);
 if (IS_ERR(drvdata->regmap)) {
  status = PTR_ERR(drvdata->regmap);
  dev_err(&pdev->dev, "%s: Failed to allocate regmap: %d\n",
   __func__, status);
  return status;
 }

 codec_clk = devm_clk_get(&pdev->dev, "mclk");
 if (IS_ERR(codec_clk)) {
  dev_err(&pdev->dev, "get mclk failed\n");
  err = PTR_ERR(codec_clk);
  status = -1;
  goto err_exit;
 }
 clk_prepare_enable(codec_clk);

 dev_dbg(&pdev->dev, "%s: Register codec.\n", __func__);
 status = snd_soc_register_codec(&pdev->dev, &ab8500_codec_driver,
    ab8500_codec_dai,
    ARRAY_SIZE(ab8500_codec_dai));






err_exit:
 if (status < 0)
  dev_err(&pdev->dev,
   "%s: Error: Failed to register codec (%d).\n",
   __func__, status);
 return status;
}

static int ab8500_codec_driver_remove(struct platform_device *pdev)
{
 dev_dbg(&pdev->dev, "%s Enter.\n", __func__);

 snd_soc_unregister_codec(&pdev->dev);

 return 0;
}

static const struct of_device_id ab8500_dt_ids[] = {
 { .compatible = "ti,i2scodec", },
 { }
};
MODULE_DEVICE_TABLE(of, ab8500_dt_ids);

static struct platform_driver ab8500_codec_platform_driver = {
 .driver = {
  .name = "ab8500-codec",
  .of_match_table = ab8500_dt_ids,
 },
 .probe = ab8500_codec_driver_probe,
 .remove = ab8500_codec_driver_remove,
 .suspend = NULL,
 .resume = NULL,
};
module_platform_driver(ab8500_codec_platform_driver);

MODULE_LICENSE("GPL v2");
