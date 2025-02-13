/* NUVOTON NAU7802 ADC
 *
 * Copyright (c) 2025 https://github.com/mauro-medina10
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(nau7802, CONFIG_ADC_LOG_LEVEL);

#define DT_DRV_COMPAT nuvoton_nau7802_adc

//----------------------------------------------------------------------
//	Definitions
//----------------------------------------------------------------------

#define NAU7802_INT_DEFAULT_PIN 						ADC24_INT_PIN

#define ADC_I2C_ADDR 									0x2A
#define NAU7802_ID 										0x0F

#define NAU7802_AVERAGE_TIMEOUT 						2000
#define NAU7802_CAL_TIME								NAU7802_AVERAGE_TIMEOUT
#define NAU7802_SETTLE_SAMPLES 							6

#define NAU7802_DRDY2_MODE								1 // DRDY pin

// #define CONFIG_NAU7802_SINGLE_CHN

#define NAU7802_MAX_CNT 								(1 << 23)
#define SHIFT_TEMP(val) 								((val >> 0))
//TEMPERATURE MODE
#define NAU7802_TEMP_REF_C 								25
#define NAU7802_TEMP_REF_MV 							109
#define NAU7802_TEMP_DELTA_UV 							360
#define NAU7802_TEMP_OFFSET								25.919141f

// Temp buffer size
#define TEMP_MAX_SAMPLES								20

// Tiempo de vida
#define NAU7802_ALIVE_TIME_MS							500

static int nau7802_channel_setup(const struct device *dev,
                                const struct adc_channel_cfg *channel_cfg);

static int nau7802_read(const struct device *dev,
                       const struct adc_sequence *sequence);
//Register Map
typedef enum
{
	NAU7802_PU_CTRL      = 0x00,
	NAU7802_CTRL1,
	NAU7802_CTRL2,
	NAU7802_OCAL1_B2,
	NAU7802_OCAL1_B1,
	NAU7802_OCAL1_B0,
	NAU7802_GCAL1_B3,
	NAU7802_GCAL1_B2,
	NAU7802_GCAL1_B1,
	NAU7802_GCAL1_B0,
	NAU7802_OCAL2_B2,
	NAU7802_OCAL2_B1,
	NAU7802_OCAL2_B0,
	NAU7802_GCAL2_B3,
	NAU7802_GCAL2_B2,
	NAU7802_GCAL2_B1,
	NAU7802_GCAL2_B0,
	NAU7802_I2C_CONTROL,
	NAU7802_ADCO_B2,
	NAU7802_ADCO_B1,
	NAU7802_ADCO_B0,
	NAU7802_ADC          = 0x15,	//Shared ADC and OTP 32:24
	NAU7802_OTP_B1,					//OTP 23:16 or 7:0?
	NAU7802_OTP_B0,					//OTP 15:8
	NAU7802_PGA          = 0x1B,
	NAU7802_PGA_PWR      = 0x1C,
	NAU7802_DEVICE_REV   = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
	NAU7802_PU_CTRL_RR = 0,
	NAU7802_PU_CTRL_PUD,
	NAU7802_PU_CTRL_PUA,
	NAU7802_PU_CTRL_PUR,
	NAU7802_PU_CTRL_CS,
	NAU7802_PU_CTRL_CR,
	NAU7802_PU_CTRL_OSCS,
	NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

//Bits within the CTRL1 register
typedef enum
{
	NAU7802_CTRL1_GAIN     = 2,
	NAU7802_CTRL1_VLDO     = 5,
	NAU7802_CTRL1_DRDY_SEL = 6,
	NAU7802_CTRL1_CRP      = 7,
} CTRL1_Bits;

//Bits within the CTRL2 register
typedef enum
{
	NAU7802_CTRL2_CALMOD    = 0,
	NAU7802_CTRL2_CALS      = 2,
	NAU7802_CTRL2_CAL_ERROR = 3,
	NAU7802_CTRL2_CRS       = 4,
	NAU7802_CTRL2_CHS       = 7,
} CTRL2_Bits;

//Bits within the PGA register
typedef enum
{
	NAU7802_PGA_CHP_DIS    = 0,
	NAU7802_PGA_INV        = 3,
	NAU7802_PGA_BYPASS_EN,
	NAU7802_PGA_OUT_EN,
	NAU7802_PGA_LDOMODE,
	NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

//Bits within the PGA PWR register
typedef enum
{
	NAU7802_PGA_PWR_PGA_CURR       = 0,
	NAU7802_PGA_PWR_ADC_CURR       = 2,
	NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
	NAU7802_PGA_PWR_PGA_CAP_EN     = 7,
} PGA_PWR_Bits;

//Bits within the I2C control register
typedef enum
{
	NAU7802_PU_I2C_CTRL_BGPCP = 0,
	NAU7802_PU_I2C_CTRL_TS,
	NAU7802_PU_I2C_CTRL_BOPGA,
	NAU7802_PU_I2C_CTRL_WPD,
	NAU7802_PU_I2C_CTRL_SI,
	NAU7802_PU_I2C_CTRL_SPE,
	NAU7802_PU_I2C_CTRLL_FRD,
	NAU7802_PU_I2C_CTRL_CRSD,
} PU_I2C_CTRL_Bits;

//Allowed Low drop out regulator voltages
typedef enum
{
	NAU7802_LDO_2V4 = 0x07, 		// 0b111,
	NAU7802_LDO_2V7 = 0x06, 		// 0b110,
	NAU7802_LDO_3V0 = 0x05, 		// 0b101,
	NAU7802_LDO_3V3 = 0x04, 		// 0b100,
	NAU7802_LDO_3V6 = 0x03, 		// 0b011,
	NAU7802_LDO_3V9 = 0x02, 		// 0b010,
	NAU7802_LDO_4V2 = 0x01, 		// 0b001,
	NAU7802_LDO_4V5 = 0x00, 		// 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
	NAU7802_GAIN_128 = 0x07,		// 0b111,
	NAU7802_GAIN_64  = 0x06,		// 0b110,
	NAU7802_GAIN_32  = 0x05,		// 0b101,
	NAU7802_GAIN_16  = 0x04,		// 0b100,
	NAU7802_GAIN_8   = 0x03,		// 0b011,
	NAU7802_GAIN_4   = 0x02,		// 0b010,
	NAU7802_GAIN_2   = 0x01,		// 0b001,
	NAU7802_GAIN_1   = 0x00,		// 0b000,
} NAU7802_Gain_Values;

//Allowed samples per second
typedef enum
{
	NAU7802_SPS_320 = 0x07,			// 0b111,
	NAU7802_SPS_80  = 0x03,			// 0b011,
	NAU7802_SPS_40  = 0x02,			// 0b010,
	NAU7802_SPS_20  = 0x01,			// 0b001,
	NAU7802_SPS_10  = 0x00,			// 0b000,
} NAU7802_SPS_Values;

//Select between channel values
typedef enum
{
	NAU7802_CHANNEL_1 = 0,
	NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

//Calibration state
typedef enum
{
	NAU7802_CAL_SUCCESS		= 0,
	NAU7802_CAL_IN_PROGRESS = 1,
	NAU7802_CAL_FAILURE		= 2,
} NAU7802_Cal_Status;

typedef enum
{
	NAU7802_S_CHN_MODE = 0,
	NAU7802_D_CHN_MODE,
} Nau7802_chn_mode;

struct nau7802_config 
{
    // I2C device
	struct i2c_dt_spec  i2c;
	// Configuracion inicial
	uint8_t             channel;
	uint8_t             ldo_mode;
	uint8_t             pga_gain;
	uint8_t             sps_mode;
	Nau7802_chn_mode    chn_mode;	    // Single or Double channel
	
#ifdef ADC_NAU7802_TRIGGER
	struct gpio_dt_spec alert_rdy;
#endif

#ifdef ADC_NAU7802_TEMP
	uint16_t            ldo_mode_mv;
	NAU7802_Gain_Values gain_mode;
#endif
};

struct nau7802_data
{
	volatile uint8_t    irq_drdy_flg;   // Internal irq drdy flag
	volatile int32_t    irq_data;	    // ADC data

	uint8_t             pwr_dwn_flg;    // Device power down flag
	uint8_t             cal_status;	    // Calibration OK
#ifdef ADC_NAU7802_TRIGGER
	struct gpio_callback gpio_cb;
	struct k_work work;
#endif
};

static const struct adc_driver_api nau7802_api = {
    .channel_setup = nau7802_channel_setup,
    .read = nau7802_read,
};

static int nau7802_getRegister(const struct device *dev, uint8_t reg, uint16_t *value);
static int nau7802_setRegister(const struct device *dev, uint8_t reg, uint8_t value);
static int nau7802_getSamples(const struct device *dev, uint8_t *value);
static int nau7802_setBit(const struct device *dev, uint8_t bitNumber, uint8_t reg);
static int nau7802_clearBit(const struct device *dev, uint8_t bitNumber, uint8_t reg);
static int nau7802_getBit(const struct device *dev, uint8_t bitNumber, uint8_t reg, uint16_t *value);

static int nau7802_setGain(const struct device *dev, uint8_t gainValue);
static int nau7802_setLDO(const struct device *dev, uint8_t ldoValue);
static int nau7802_powerUp(const struct device *dev);
static int nau7802_setChannel(const struct device *dev, uint8_t channelNumber);
static int nau7802_setSampleRate(const struct device *dev, NAU7802_SPS_Values rate);
static int nau7802_waitForCalibrateAFE(const struct device *dev, uint32_t timeout_ms);
static int nau7802_initial_config(const struct device *dev, Nau7802_chn_mode chn_mode);
static int nau7802_start_conversions(const struct device *dev);
static int nau7802_stop_conversions(const struct device *dev);
static int nau7802_waitForCalibrateAFE(const struct device *dev, uint32_t timeout_ms);
static int nau7802_setChannel(const struct device *dev, uint8_t channelNumber);
static int nau7802_LDO_on(const struct device *dev);
static int nau7802_LDO_off(const struct device *dev);
static int nau7802_powerDown(const struct device *dev);
static int nau7802_reset(const struct device *dev);
static int nau7802_setLDO(const struct device *dev, uint8_t ldoValue);
static int nau7802_setGain(const struct device *dev, uint8_t gainValue);
int32_t nau7802_getReading(const struct device *dev);
static int nau7802_check_chip_id(const struct device *dev);
static int nau7802_beginCalibrateAFE(const struct device *dev);
static int nau7802_calibrateAFE(const struct device *dev);
//---------------------------------------------------------------------------------------//
//  REGISTER IO
//---------------------------------------------------------------------------------------//

/**
 * @brief Send a given value to be written to given address
 * 
 * @param reg register addr
 * @param value value to write
 * 
 * @return 0 if successful
 */
static int nau7802_setRegister(const struct device *dev, uint8_t reg, uint8_t value)
{
    const struct nau7802_config *config = dev->config;
    int ret = 0;
    uint8_t tmp[2] = {reg, value};

    ret = i2c_write_dt(&config->i2c, tmp, sizeof(tmp));

	return ret;
}

/**
 * @brief Get contents of a register
 * 
 * @param reg register addr
 * @param value pointer where the value is stored
 * 
 * @return 0 if successful
 */
static int nau7802_getRegister(const struct device *dev, uint8_t reg, uint16_t *value)
{
    const struct nau7802_config *config = dev->config;
    int ret = 0;
	uint8_t tmp[2] = {0};
	
    if(!value) return -1;

	ret = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), tmp, sizeof(tmp));
    if (ret) {
		return ret;
	}

	*value = sys_get_be16(tmp);

	return 0;
}

/**
 * @brief Get data samples
 * 
 * @param value 
 * 
 * @return 0 if successful
 * 
 */
static int nau7802_getSamples(const struct device *dev, uint8_t *value)
{	
	const struct nau7802_config *config = dev->config;
    uint8_t tmp[3] = {0};
	uint8_t reg = NAU7802_ADCO_B2;
    int ret = 0;

    ret = i2c_write_read_dt(&config->i2c, (const void*)&reg, sizeof(reg), (void*)tmp, sizeof(tmp));

	if(ret)
    {	
		return ret; //Error
	}
	
    *value = sys_get_be24(tmp);
	
    return ret;		
}

/**
 * @brief Mask & set a given bit within a register
 * 
 * @param bitNumber 
 * @param reg 
 * 
 * @return 0 if successful
 * 
 */
static int nau7802_setBit(const struct device *dev, uint8_t bitNumber, uint8_t reg)
{
	int ret = 0;
    int16_t value = 0;
    
    ret = nau7802_getRegister(dev, reg, &value);
	if(ret)
	{
		return ret; //Error
	}

	value |= (1 << bitNumber); //Set this bit
	
    return (nau7802_setRegister(dev, reg, (uint8_t)value));
}

/**
 * @brief Mask & clear a given bit within a register
 * 
 * @param bitNumber 
 * @param reg 
 * 
 * @return 0 if successful
 * 
 */
static int nau7802_clearBit(const struct device *dev, uint8_t bitNumber, uint8_t reg)
{
    int ret = 0;
	int16_t value = 0;
    
    ret = nau7802_getRegister(dev, reg, &value);
	if(ret)
	{
		return ret; //Error
	}

	value &= ~(1 << bitNumber);		//Set this bit

	return (nau7802_setRegister(dev, reg, (uint8_t)value));
}

/**
 * @brief Return a given bit within a register 
 * 
 * @param bitNumber 
 * @param reg 
 * 
 * @return 0 if successful
 * 
 */
static int nau7802_getBit(const struct device *dev, uint8_t bitNumber, uint8_t reg, uint16_t *value)
{
    int ret = 0;
    
    ret = nau7802_getRegister(dev, reg, value);
	if(ret)
	{
		return ret; //Error
	}

	*value &= (1 << bitNumber);		//Clear all but this bit

	return ret;
}

//---------------------------------------------------------------------------------------//
//  API
//---------------------------------------------------------------------------------------//

/**
 * @brief Sets up the NAU7802 for basic function
 * 
 * @return 0 if successful
 * 
 */
int nau7802_init(const struct device *dev)
{
	const struct nau7802_config *config = dev->config;
	const struct i2c_dt_spec *i2c = &config->i2c;
    int ret = 0;

	//Verify I2C
	if (!i2c_is_ready_dt(i2c)) {
		LOG_ERR("Bus not ready");
		return -EINVAL;
	}
	// Power on analog and digital sections of the scale
	ret = nau7802_powerUp(dev);
	// CHIP-ID
	ret |= nau7802_check_chip_id(dev);
	if(ret)
	{
		return ret;
	}
	// Initial config
	ret |= nau7802_initial_config(dev, config->chn_mode);
	// Re-cal analog front end when we change gain, sample rate, or channel
	ret |= nau7802_calibrateAFE(dev);	
	if(ret)
	{
		return ret;
	}

#ifdef CONFIG_NAU7802_STABLE	
	ret = nau7802_estabilization(dev);
#endif	
	return ret;
}

static int nau7802_channel_setup(const struct device *dev,
                                const struct adc_channel_cfg *channel_cfg) {
    // Validate channel ID and gain here
    return 0;
}

static int nau7802_read(const struct device *dev,
                       const struct adc_sequence *sequence) {
    // Implement ADC read logic using nau7802_getReading()
    return 0;
}

/**
 * @brief Configuracion inicial del ADC
 * 
 * @param chn_mode 
 * @param result 
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_initial_config(const struct device *dev, Nau7802_chn_mode chn_mode)
{
    const struct nau7802_config *config = dev->config;
    int ret = 0;

	// Canal
	ret |= nau7802_setChannel(dev, config->channel);
#ifdef ADC_NAU7802_LDO_OFF
	ret |= nau7802_LDO_off(dev);
#else
	//Set LDO to 3.3V note: 2v4 not recommended (doesnt work)
	ret |= nau7802_setLDO(dev, config->ldo_mode); 
#endif
#ifdef ADC_NNAU7802_I2C_PULL_S
	ret |= nau7802_setBit(dev, NAU7802_PU_I2C_CTRL_SPE, NAU7802_I2C_CONTROL);
#endif
#ifdef ADC_NNAU7802_I2C_WEAK_PULLUP_OFF
	// Turns off 50k pullup resistor
	ret |= nau7802_setBit(dev, NAU7802_PU_I2C_CTRL_WPD, NAU7802_I2C_CONTROL);	
#endif
	//Set gain
	ret |= nau7802_setGain(dev, config->pga_gain);							
	//Set samples per second 
	ret |= nau7802_setSampleRate(dev, config->sps_mode); 				
	//Turn off CLK_CHP. From 9.1 power on sequencing.
	ret |= nau7802_setRegister(dev, NAU7802_ADC, 0x30);							
#ifdef ADC_NAU7802_SINGLE_CHN
	if(chn_mode == NAU7802_S_CHN_MODE)
	{
		//Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note
		ret |= nau7802_setBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);	
	}
#endif	
#ifdef ADC_NAU7802_LOW_POWER
	ret |= low_accuracy_set(dev);
#endif	
	return ret;
}

/**
 * @brief Comienza el ADC 
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_start_conversions(const struct device *dev)
{
	
	nau7802_getReading(dev);

	return nau7802_setBit(dev, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

/**
 * @brief Detiene el ADC
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_stop_conversions(const struct device *dev)
{
	
	return nau7802_clearBit(dev, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

#ifdef ADC_NAU7802_AVAILABLE
/**
 * @brief Returns true if conversion is complete
 			mode 0 : Cycle Ready bit is set
 			mode 1 : DataReady pin high
 *			
 * @param mode 
 * 
 * @return 1 if data ready
 *  
 */
static int nau7802_available(const struct device *dev, uint8_t mode)
{
    int ret = 0;
    uint16_t value = 0;

	if(mode == 0)
	{
        ret = nau7802_getBit(dev, NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL, &value);
        if(ret)
        {
            return 0;
        }else
        {
		    return value;
        }
	}
#ifdef ADC_NAU7802_TRIGGER

#endif
    return 0;
}
#endif

/**
 * @brief Calibrates the NAU7802.
 * 
 * @details Takes approximately 344ms to calibrate; wait up to 1000ms.
 			It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_calibrateAFE(const struct device *dev)
{
	int ret = 0;

	ret = nau7802_beginCalibrateAFE(dev);
    if(ret)
    {
        return ret;
    }

	ret = nau7802_waitForCalibrateAFE(dev, NAU7802_CAL_TIME);

	return(ret); 
}

/**
 * @brief Comienza la calibracion del ADC
 * 
 */
static int nau7802_beginCalibrateAFE(const struct device *dev)
{
	return nau7802_setBit(dev, NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

/**
 * @brief Verifies calibration status
 * 
 * @return NAU7802_Cal_Status 
 */
NAU7802_Cal_Status nau7802_calAFEStatus(const struct device *dev)
{
    int ret = 0;
    uint16_t value = 0;

    ret = nau7802_getBit(dev, NAU7802_CTRL2_CALS, NAU7802_CTRL2, &value);
	if (ret == 0 && value)
	{
		return NAU7802_CAL_IN_PROGRESS;
	}

    ret = nau7802_getBit(dev, NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2, &value);
	if (ret == 0 && value)
	{
		return NAU7802_CAL_FAILURE;
	}

	// Calibration passed
	return NAU7802_CAL_SUCCESS;
}

/**
 * @brief Wait for asynchronous AFE calibration to complete with optional timeout.
 * 
 * @details If timeout is not specified (or set to 0), then wait indefinitely.
			Returns true if calibration completes succsfully, otherwise returns false.
 * 
 * @param timeout_ms 
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_waitForCalibrateAFE(const struct device *dev, uint32_t timeout_ms)
{	
	struct nau7802_data *data = dev->data;
	volatile NAU7802_Cal_Status cal_ready;
	
	cal_ready = nau7802_calAFEStatus(dev);

	while (cal_ready == NAU7802_CAL_IN_PROGRESS)
	{
		if ((timeout_ms-- > 0))
		{
			break;
		}

		cal_ready = nau7802_calAFEStatus(dev);
		k_msleep(1);
	}

	if (cal_ready == NAU7802_CAL_SUCCESS)
	{
		data->cal_status = 1;

		return 0;
	}
	
	return -ETIME;
}

/**
 * @brief Sets sampling rate.
 * 
 * @note 10, 20, 40, 80, and 320 samples per second is available
 * 
 * @param rate 
 * @return true 
 * @return false 
 */
static int nau7802_setSampleRate(const struct device *dev, NAU7802_SPS_Values rate)
{
	int ret = 0;
	struct nau7802_data *data = dev->data;
	int16_t value = 0;

    ret = nau7802_getRegister(dev, NAU7802_CTRL2, &value);
	if(ret)
	{
		return ret; 			//Error
	}

	value &= 0x8F;				//Clear CRS bits
	value |= rate << 4;			//Mask in new CRS bits

	ret = nau7802_setRegister(dev, NAU7802_CTRL2, (uint8_t)value);
	if(ret) 
    {
        return ret; // error
    }
	
    data->cal_status = 0;
	
    return 0;
}

/**
 * @brief Setea canal del ADC
 * 
 * @param channelNumber 
 * @return true 
 * @return false 
 */
static int nau7802_setChannel(const struct device *dev, uint8_t channelNumber)
{
	int ret = 0;
	struct nau7802_data *data = dev->data;

	if (channelNumber == NAU7802_CHANNEL_1)
	{
#ifdef ADC_NAU7802_SINGLE_CHN		
		if(config->chn_mode == NAU7802_S_CHN_MODE)
		{
			nau7802_setBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);
		}
#endif
		ret = nau7802_clearBit(dev, NAU7802_CTRL2_CHS, NAU7802_CTRL2);	//Channel 1 (default)
	}
	else if (channelNumber == NAU7802_CHANNEL_2)
	{
#ifdef ADC_NAU7802_SINGLE_CHN	
		if(config->chn_mode == NAU7802_S_CHN_MODE)
		{
			nau7802_clearBit(dev, NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR);
		}		
#endif
		 ret = nau7802_setBit(dev, NAU7802_CTRL2_CHS, NAU7802_CTRL2);		//Channel 2
	}

	if(ret)
	{
	    return ret; // Error
	}

    data->cal_status = 0;

    return 0;
}

/**
 * @brief Turns internal LDO on
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_LDO_on(const struct device *dev)
{
    struct nau7802_data *data = dev->data;

	int ret = nau7802_setBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);

	data->cal_status = 0;

	return ret;
}

/**
 * @brief Turns internal LDO off
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_LDO_off(const struct device *dev)
{
    struct nau7802_data *data = dev->data;
	
    int ret = nau7802_clearBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);

	data->cal_status = 0;

	return ret;
}

/**
 * @brief Power up digital and analog sections of scale
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_powerUp(const struct device *dev)
{
    struct nau7802_data *data = dev->data;
    int ret = 0;
    uint16_t tmp = 0;

	nau7802_clearBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	nau7802_clearBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

	k_msleep(2);

	nau7802_setBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	nau7802_setBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

	//Wait for Power Up bit to be set - takes approximately 200us
	uint8_t counter = 0;
	
	while (1)
	{
        ret = nau7802_getBit(dev, NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL, &tmp);
		if (ret == 0 && tmp == 1)
		{
			break;				//Good to go
		}
		
		k_msleep(1);
		
		if (counter++ > 100)
		{
			return -ETIME;		//Error
		}
	}
	
	data->cal_status = 0;

	return 0;
}

/**
 * @brief Puts ADC into low-power mode
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_powerDown(const struct device *dev)
{
    struct nau7802_data *data = dev->data;
	int ret = 0;

	ret = nau7802_clearBit(dev, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	ret |= nau7802_clearBit(dev, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);
	if(ret)
	{
	    return ret;
	}

    data->pwr_dwn_flg = 1;
    data->cal_status = 0;

    return 0;
}

/**
 * @brief Resets all registers to Power Of Defaults 
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_reset(const struct device *dev)
{
    struct nau7802_data *data = dev->data;

	nau7802_setBit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);				//Set RR
		
	k_msleep(5);
	
	data->cal_status = 0;

	return nau7802_clearBit(dev, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);	//Clear RR to leave reset state
}

/**
 * @brief Set the internal Low-Drop-Out voltage regulator to a given value
 * 
 * @note 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
 * 
 * @param ldoValue 
 * @return true 
 * @return false 
 */
static int nau7802_setLDO(const struct device *dev, uint8_t ldoValue)
{
    struct nau7802_data *data = dev->data;
    int ret = 0;

	if (ldoValue > 0x07)
	{
		ldoValue = 0x07;		//Error check
	}
	
	switch (ldoValue)
	{
		case NAU7802_LDO_2V4:
#ifdef ADC_NAU7802_TEMP			
			config->ldo_mode_mv = 2400;
#endif			
			break;
		case NAU7802_LDO_2V7:
#ifdef ADC_NAU7802_TEMP			
			config->ldo_mode_mv = 2700;
#endif			
			break;
		case NAU7802_LDO_3V0:
#ifdef ADC_NAU7802_TEMP		
			config->ldo_mode_mv = 3000;
#endif			
			break;
		case NAU7802_LDO_3V3:
#ifdef ADC_NAU7802_TEMP		
			config->ldo_mode_mv = 3300;
#endif			
			break;
		case NAU7802_LDO_3V6:
#ifdef ADC_NAU7802_TEMP		
			config->ldo_mode_mv = 3600;
#endif			
			break;
		case NAU7802_LDO_3V9:
#ifdef ADC_NAU7802_TEMP		
			config->ldo_mode_mv = 3900;
#endif			
			break;
		case NAU7802_LDO_4V2:
#ifdef ADC_NAU7802_TEMP		
			config->ldo_mode_mv = 4200;
#endif			
			break;
		case NAU7802_LDO_4V5:
#ifdef ADC_NAU7802_TEMP		
			config->ldo_mode_mv = 4500;
#endif			
			break;
		default:
#ifdef ADC_NAU7802_TEMP			
			config->ldo_mode_mv = 3000;
#endif			
			break;
	}
	//Set the value of the LDO
	int16_t value = 0;
    
    ret = nau7802_getRegister(dev, NAU7802_CTRL1, &value);
	if(ret)
	{
		return ret; //Error
	}

	value &= 0xC7;		//Clear LDO bits
	value |= ldoValue << 3;		//Mask in new LDO bits

	nau7802_setRegister(dev, NAU7802_CTRL1, (uint8_t)value);

	data->cal_status = 0;
	
	return (nau7802_setBit(dev, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL));	//Enable the internal LDO
}

/**
 * @brief Setea la ganancia
 * 
 * @note x1, x2, x4, x8, x16, x32, x64, x128 are avaialable
 * 
 * @param gainValue 
 * @return true 
 * @return false 
 */
static int nau7802_setGain(const struct device *dev, uint8_t gainValue)
{
    struct nau7802_data *data = dev->data;
	int ret = 0;

	if (gainValue > 0x07)
	{
		gainValue = 0x07;		//Error check
	}
#ifdef ADC_NAU7802_TEMP
	config->gain_mode = gainValue;
#endif
	int16_t value = 0;
    
    ret = nau7802_getRegister(dev, NAU7802_CTRL1, &value);
	if(ret)
	{
		return ret; //Error
	}

	value &= 0xF8;		//Clear gain bits
	value |= gainValue;			//Mask in new bits

	ret = nau7802_setRegister(dev, NAU7802_CTRL1, (uint8_t)value);

	if(ret) 
    {
	    return ret;
    }

    data->cal_status = 0;
    
    return 0;
}

#ifdef ADC_NAU7802_REV_CODE
/**
 * @brief Get the revision code of this IC
 * 
 * @return uint8_t 
 */
uint8_t nau7802_getRevisionCode(const struct device *dev)
{
    int ret = 0;
	int16_t revisionCode = 0;
    
    ret = nau7802_getRegister(dev, NAU7802_DEVICE_REV, &value);
	if(ret)
	{
		return 0; //Error
	}

	return (uint8_t)(revisionCode & 0x0F);
}
#endif

/**
 * @brief Returns 24-bit reading. Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
 * 
 * @return int32_t Data
 */
int32_t nau7802_getReading(const struct device *dev)
{
	uint32_t aux = 0;
	int32_t value = 0;
	uint8_t values[3] = {0};

	nau7802_getSamples(dev, values);

	aux = (uint32_t)(values[0] << 16);
	
	aux |= (uint32_t)(values[1] << 8);
	
	aux |= (uint32_t)(values[2]);
	
	value = (int32_t) (aux << 8);
	
	int32_t val = (value >> 8);
	
	return val;
}

#ifdef CONFIG_ADC_ASYNC
/**
 * @brief Set Int pin to be high when data is ready (default) 
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_setIntPolarityHigh(const struct device *dev)
{
	return (nau7802_clearBit(dev, NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}

/**
 * @brief Set Int pin to be low when data is ready
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_setIntPolarityLow(const struct device *dev)
{
	return (nau7802_setBit(dev, NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}

/**
 * @brief Register adc interrupt callback
 *
 * @param[in] pin  CPU pin connected to DRDY adc pin.
 *						 					
 * @param[in] cb   Callback for data ready events
 *						
 * @return Registration status.
 * @retval -1 Passed parameters were invalid
 * @retval 0 The callback registration is completed successfully
 */
static int nau7802_drdy_cb_register(const struct device *dev, const uint32_t pin, ext_irq_cb_t cb){
	
	int ret = false;

	// Clear EIC irq flag
	hri_eic_clear_INTFLAG_reg(EIC, 0xFFFF);

	if(cb != dev->config->irq_user_cb)
	{
		dev->config->irq_pin = pin;
		dev->config->irq_user_cb = cb;
		
		ret = ext_irq_register(pin, cb, dev) == 0;
	}

	return ret;
}
/**
 * @brief Habilita la interrupcion del ADC
 * 
 * @return int32_t 
 */
int32_t nau7802_enable_irq(const struct device *dev){
	
	// Clear EIC irq flag
	hri_eic_clear_INTFLAG_reg(EIC, 0xFFFF);

	if(dev->config->irq_pin != 0xFFFFFFFF){

		nau7802_getReading(dev);
	
		return ext_irq_register(dev->config->irq_pin, (ext_irq_cb_t)dev->config->irq_user_cb, dev);
	}else
	{
		return ext_irq_register(NAU7802_INT_DEFAULT_PIN, (ext_irq_cb_t)nau7802_irq_cb, dev);
	}	
	return -1;
}

/**
 * @brief Deshabilita la interrupcion del ADC
 * 
 * @return int32_t 
 */
int32_t nau7802_disable_irq(const struct device *dev){
	
	if(dev->config->irq_pin != 0xFFFFFFFF){
		
		return ext_irq_register(dev->config->irq_pin, NULL, dev);
	}else
	{
		return ext_irq_register(NAU7802_INT_DEFAULT_PIN, NULL, dev);
	}
	return -1;
}

/**
 * @brief Habilita la interrupcion interna del ADC.
 * 
 * @return int32_t 
 */
int32_t nau7802_internal_irq_enable(const struct device *dev){
	
	// Clear EIC irq flag
	hri_eic_clear_INTFLAG_reg(EIC, 0xFFFF);

	if(dev->config->irq_pin != 0xFFFFFFFF){
		
		nau7802_getReading(dev);

		return ext_irq_register(dev->config->irq_pin, nau7802_irq_cb, dev);
	}else
	{
		nau7802_getReading(dev);

		return ext_irq_register(NAU7802_INT_DEFAULT_PIN, nau7802_irq_cb, dev);
	}
	return -1;
}

/**
 * @brief Deshabilita la interrupcion interna del ADC.
 * 
 * @return int32_t 
 */
int32_t nau7802_internal_irq_disable(const struct device *dev){
	
	return nau7802_disable_irq(dev);
}
#endif
/**
 * @brief Checks for a 0x0F ID.
 * 
 * @return 0 if successful
 *  
 */
static int nau7802_check_chip_id(const struct device *dev){
	int ret = 0;
    uint16_t value = 0;

	ret = nau7802_getRegister(dev, NAU7802_DEVICE_REV, &value);
    if(ret)
    {
        return -EINVAL;
    }

    return !(value == NAU7802_ID);
}

#ifdef ADC_NAU7802_TEMP
/**
 * @brief Setea el canal interno del ADC para medir temperatura y calibra.
 * 
 * @details Sensor de temperatura:	
			Salida: 109 mV a 25�C
			Delta: 360 uV/�C (relativo 25�c)
 *		
 * 
 * @param tmp_sens true: ganancia x1; false: ganancia x128
 * @return true 
 * @return false 
 */
static int nau7802_temperature_input_set(const struct device *dev, int tmp_sens){

	int ret = false;

	if(tmp_sens){

		ret = (nau7802_setGain(dev, NAU7802_GAIN_1) == 0);

		ret |= (nau7802_setBit(dev, NAU7802_PU_I2C_CTRL_TS, NAU7802_I2C_CONTROL) == 0);
	}else{

		ret = (nau7802_setGain(dev, NAU7802_GAIN_128) == 0);

		ret |= (nau7802_clearBit(dev, NAU7802_PU_I2C_CTRL_TS, NAU7802_I2C_CONTROL) == 0);
	}

	espera_ms(10);
	
	ret |= (nau7802_calibrateAFE(dev) == 0);

	return ret;
}

/**
 * @brief Calcula la temperatura a partir de una medicion del ADC.
 * 
 * @param adc_data 
 * @return double 
 */
static double temperature_calc(const struct device *dev, double adc_data)
{
	//Temperatura en mV
	double temp_aux = (double)(adc_data * dev->config->ldo_mode_mv) 
								/ NAU7802_MAX_CNT; 
	//delta temp en uV
	temp_aux = (double)(temp_aux - NAU7802_TEMP_REF_MV) * 1000; 
	//delta temp en �C
	temp_aux = (double)(temp_aux / NAU7802_TEMP_DELTA_UV); 
	//Temperatura en �C
	temp_aux = NAU7802_TEMP_REF_C + temp_aux;		

	return temp_aux + NAU7802_TEMP_OFFSET;	
}

/**
 * @brief Obtiene el valor de temperatura.
 * 
 * @param average_samples cantidad de muestras a promediar
 * @return double 
 */
double nau7802_temperature_get(const struct device *dev, uint8_t average_samples){

	double temp_aux;
	int32_t data_aux;
	int ret = 0;
#ifdef ADC_NAU7802_MEDIAN_TEMP
	int temp_data[TEMP_MAX_SAMPLES] = {0};
	uint8_t samples = MIN(TEMP_MAX_SAMPLES, average_samples);
#else
	int32_t temp_accum = 0;
	uint8_t samples = average_samples;	
#endif
	nau7802_internal_irq_enable(dev);
	dev->config->irq_drdy_flg = false;

	for(uint8_t i = 0; i < samples; i++){
		
		ret = get_irq_data(dev, &data_aux);
		if(ret != 0) return 0;
#ifdef ADC_NAU7802_MEDIAN_TEMP
		temp_data[i] = (int)data_aux; 
#else
		temp_accum += data_aux;
#endif
		dev->config->irq_drdy_flg = false;
	}

	nau7802_internal_irq_disable(dev);

#ifdef ADC_NAU7802_MEDIAN_TEMP
	temp_aux = (double)(util_get_median_int(temp_data, samples) >> 0);
#else
	temp_aux = (double)(temp_accum / samples);	//Promedio adc
#endif	

	return temperature_calc(dev, temp_aux);
}

/**
 * @brief Habilita el modo de medicion de termperatura
 * 
 */
void nau7802_temperature_enable(const struct device *dev)
{
	nau7802_temperature_input_set(dev, true);
	nau7802_disable_irq(dev);
}

/**
 * @brief Deshabilita el modo de medicion de termperatura
 * 
 */
void nau7802_temperature_disable(const struct device *dev)
{
	nau7802_temperature_input_set(dev, false);
	nau7802_enable_irq(dev);
}
#endif

// Update device instantiation macro
#define ADC_NAU7802_INST_DEFINE(n)                                              \
    static const struct nau7802_config config_##n = {                           \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                         \
        .channel = DT_INST_PROP_OR(n, channel, 0),                                    \
        .ldo_mode = DT_INST_PROP_OR(n, ldo_voltage, 0),                               \
        .pga_gain = DT_INST_PROP_OR(n, gain, 0),                                      \
        .sps_mode = DT_INST_PROP_OR(n, sps, 0),                                       \
    };                                                                          \
    static struct nau7802_data data_##n;                                        \
    DEVICE_DT_INST_DEFINE(n, nau7802_init, NULL, &data_##n, &config_##n,        \
                          POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &nau7802_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_NAU7802_INST_DEFINE)