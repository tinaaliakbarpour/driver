/***************************************************************************//**
 *   @file   ADF4351.h
 *   @brief  Header file of ADF4351 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/
#ifndef __ADF4351_H__
#define __ADF4351_H__

#include <ADF4351_cfg.h>
#include <boost/cstdint.hpp>

/* Registers */
#define ADF4351_REG0	0
#define ADF4351_REG1	1
#define ADF4351_REG2	2
#define ADF4351_REG3	3
#define ADF4351_REG4	4
#define ADF4351_REG5	5

/* REG0 Bit Definitions */
#define ADF4351_REG0_FRACT(x)					(((x) & 0xFFF) << 3)
#define ADF4351_REG0_INT(x)						(((x) & 0xFFFF) << 15)

/* REG1 Bit Definitions */
#define ADF4351_REG1_MOD(x)						(((x) & 0xFFF) << 3)
#define ADF4351_REG1_PHASE(x)					(((x) & 0xFFF) << 15)
#define ADF4351_REG1_PRESCALER					(1 << 27)
#define ADF4351_REG1_PHA_ADJ                    (1 << 28)

/* REG2 Bit Definitions */
#define ADF4351_REG2_COUNTER_RESET_EN			(1 << 3)
#define ADF4351_REG2_CP_THREESTATE_EN			(1 << 4)
#define ADF4351_REG2_POWER_DOWN_EN				(1 << 5)
#define ADF4351_REG2_PD_POLARITY_POS			(1 << 6)
#define ADF4351_REG2_LDP_6ns					(1 << 7)
#define ADF4351_REG2_LDP_10ns					(0 << 7)
#define ADF4351_REG2_LDF_FRACT_N				(0 << 8)
#define ADF4351_REG2_LDF_INT_N					(1 << 8)
#define ADF4351_REG2_CHARGE_PUMP_CURR_uA(x)		(((((x)-312) / 312) & 0xF) << 9)
#define ADF4351_REG2_DOUBLE_BUFF_EN				(1 << 13)
#define ADF4351_REG2_10BIT_R_CNT(x)				((x) << 14)
#define ADF4351_REG2_RDIV2_EN					(1 << 24)
#define ADF4351_REG2_RMULT2_EN					(1 << 25)
#define ADF4351_REG2_MUXOUT(x)					((x) << 26)
#define ADF4351_REG2_NOISE_MODE(x)				((x) << 29)

/* REG3 Bit Definitions */
#define ADF4351_REG3_12BIT_CLKDIV(x)			((x) << 3)
#define ADF4351_REG3_12BIT_CLKDIV_MODE(x)		((x) << 16)
#define ADF4351_REG3_12BIT_CSR_EN				(1 << 18)
#define ADF4351_REG3_CHARGE_CANCELLATION_EN		(1 << 21)
#define ADF4351_REG3_ANTI_BACKLASH_3ns_EN		(1 << 22)
#define ADF4351_REG3_BAND_SEL_CLOCK_MODE_HIGH	(1 << 23)

/* REG4 Bit Definitions */
#define ADF4351_REG4_OUTPUT_PWR(x)				((x) << 3)
#define ADF4351_REG4_RF_OUT_EN					(1 << 5)
#define ADF4351_REG4_AUX_OUTPUT_PWR(x)			((x) << 6)
#define ADF4351_REG4_AUX_OUTPUT_EN				(1 << 8)
#define ADF4351_REG4_AUX_OUTPUT_FUND			(1 << 9)
#define ADF4351_REG4_AUX_OUTPUT_DIV				(0 << 9)
#define ADF4351_REG4_MUTE_TILL_LOCK_EN			(1 << 10)
#define ADF4351_REG4_VCO_PWRDOWN_EN				(1 << 11)
#define ADF4351_REG4_8BIT_BAND_SEL_CLKDIV(x)	((x) << 12)
#define ADF4351_REG4_RF_DIV_SEL(x)				((x) << 20)
#define ADF4351_REG4_FEEDBACK_DIVIDED			(0 << 23)
#define ADF4351_REG4_FEEDBACK_FUND				(1 << 23)

/* REG5 Bit Definitions */
#define ADF4351_REG5_LD_PIN_MODE_LOW			(0 << 22)
#define ADF4351_REG5_LD_PIN_MODE_DIGITAL		(1 << 22)
#define ADF4351_REG5_LD_PIN_MODE_HIGH			(3 << 22)

/* Specifications */
#define ADF4351_MAX_OUT_FREQ		4400000000ULL /* Hz */
#define ADF4351_MIN_OUT_FREQ		34375000 /* Hz */
#define ADF4351_MIN_VCO_FREQ		2200000000ULL /* Hz */
#define ADF4351_MAX_FREQ_45_PRESC	3000000000ULL /* Hz */
#define ADF4351_MAX_FREQ_PFD		32000000 /* Hz */
#define ADF4351_MAX_BANDSEL_CLK		125000 /* Hz */
#define ADF4351_MAX_FREQ_REFIN		250000000 /* Hz */
#define ADF4351_MAX_MODULUS			4095
#define ADF4351_MAX_R_CNT			1023

class adf4351:public boost::noncopyable
{
public:
    typedef boost::shared_ptr<adf4351> sptr;
    static sptr make(pax::spi_iface::sptr io_face,boost::uint32_t slave_num);
   virtual boost::int64_t set_freq(boost::int64_t Hz)=0;
    virtual boost::int64_t set_refin_freq(boost::int64_t Hz)=0;
    virtual boost::int32_t set_freq_resolution( boost::int32_t Hz)=0;
    virtual boost::int32_t  powerdown(bool pwd)=0;
};


class adf4351_device_t : public adf4351
{
public:
    adf4351_device_t( adf4351_io::sptr io_iface) :
         _io_iface(io_iface) {
        adf4351_setup();
    }

    /******************************************************************************/
    /************************ Functions Declarations ******************************/
    /******************************************************************************/
    /** Initializes the ADF4351. */
     boost::int32_t adf4351_setup();
    /** Sets PLL 0 frequency in Hz. */
     boost::int64_t set_freq(boost::int64_t Hz);
    /** Stores PLL 0 frequency resolution/channel spacing in Hz. */
     boost::int32_t set_freq_resolution( boost::int32_t Hz);
    /** Stores PLL 0 REFin frequency in Hz. */
     boost::int64_t set_refin_freq(boost::int64_t Hz);
    /** Powers down the PLL.  */
     boost::int32_t  powerdown(bool pwd){
         return adf4351_out_altvoltage0_powerdown(pwd);
     }

     boost::int32_t adf4351_out_altvoltage0_powerdown( boost::int32_t pwd);

    //Constants
    static const double AD9361_MAX_GAIN;
    static const double AD9361_MAX_CLOCK_RATE;
    static const double AD9361_CAL_VALID_WINDOW;
    static const double AD9361_RECOMMENDED_MAX_BANDWIDTH;
    static const double DEFAULT_RX_FREQ;
    static const double DEFAULT_TX_FREQ;

private:    //Members
    /******************************************************************************/
    /************************ Types Definitions ***********************************/
    /******************************************************************************/
    struct adf4351_platform_data
    {
        boost::uint32_t	clkin;
        boost::uint32_t	channel_spacing;
        boost::uint64_t	power_up_frequency;

        boost::uint16_t	ref_div_factor; /* 10-bit R counter */
        boost::uint8_t	    ref_doubler_en;
        boost::uint8_t	    ref_div2_en;

        boost::uint32_t    r2_user_settings;
        boost::uint32_t    r3_user_settings;
        boost::uint32_t    r4_user_settings;
        boost::int32_t gpio_lock_detect;
    };

    typedef   struct _adf4351_st
    {
        adf4351_platform_data	*pdata;
        boost::uint32_t	clkin;
        boost::uint32_t	chspc;	/* Channel Spacing */
        boost::uint32_t	fpfd;	/* Phase Frequency Detector */
        boost::uint32_t	min_out_freq;
        boost::uint32_t	r0_fract;
        boost::uint32_t	r0_int;
        boost::uint32_t	r1_mod;
        boost::uint32_t	r4_rf_div_sel;
        boost::uint32_t	regs[6];
        boost::uint32_t	regs_hw[6];
        boost::uint32_t 	val;
    } adf4351_state;

    adf4351_state st ;

    //Interfaces
    adf4351_params::sptr _client_params;
    adf4351_io::sptr     _io_iface;

private:    //Methods
    boost::int32_t adf4351_sync_config();
    boost::int32_t adf4351_tune_r_cnt(boost::uint16_t r_cnt);
    boost::uint32_t gcd( boost::uint32_t x,  boost::uint32_t y);
    boost::int64_t adf4351_set_freq(boost::uint64_t freq);
    /** Sets PLL 0 frequency in Hz assuming a constant R_CNT. */
    boost::int64_t adf4351_set_freq_const_fpfd (boost::uint64_t freq, boost::uint16_t r_cnt);

    boost::int32_t adf4351_write(boost::uint32_t data);

    //Synchronization
 //   boost::recursive_mutex  _mutex;
};


#endif // __ADF4351_H__
