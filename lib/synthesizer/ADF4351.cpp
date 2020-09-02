/***************************************************************************//**
 *   @file   ADF4351.c
 *   @brief  Implementation of ADF4351 Driver.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "ADF4351.h"
#include "ADF4351_cfg.h"
#include <exception.hpp>
#include <iostream>

static struct adf4351_platform_data adf4351_pdata_lpc =
{
    static_cast<uint32_t>(40e6),  //clkin
    static_cast<uint32_t>(10e3),      //channel_spacing
    0,          //power_up_frequency
    0,          //ref_div_factor
    0,          //ref_doubler_en
    0,          //ref_div2_en
    ADF4351_REG2_PD_POLARITY_POS |
    ADF4351_REG2_CHARGE_PUMP_CURR_uA(5000)|
    ADF4351_REG2_MUXOUT(0)|
    ADF4351_REG2_NOISE_MODE(0)|
    ADF4351_REG2_LDF_INT_N, //r2_user_settings
    ADF4351_REG3_12BIT_CLKDIV(150)|
    ADF4351_REG3_CHARGE_CANCELLATION_EN|
    ADF4351_REG3_12BIT_CLKDIV_MODE(0),      //r3_user_settings
    ADF4351_REG4_OUTPUT_PWR(0) //|
    //ADF4351_REG4_MUTE_TILL_LOCK_EN
    ,         //r4_user_settings
    -1         //gpio_lock_detect
};

/***************************************************************************//**
 * @brief Writes 4 bytes of data to ADF4351.
 *
 * @param data - Data value to write.
 *
 * @return Returns 0 in case of success or negative error code..
*******************************************************************************/
boost::int32_t adf4351_device_t::adf4351_write(boost::uint32_t data)
{
        _io_iface->write(data);
        return 1;
}

/***************************************************************************//**
 * @brief Updates the registers values.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
boost::int32_t adf4351_device_t::adf4351_sync_config()
{
        int32_t ret, i, doublebuf = 0;

        for (i = ADF4351_REG5; i >= ADF4351_REG0; i--) {
                if ((st.regs_hw[i] != st.regs[i]) ||
                        ((i == ADF4351_REG0) && doublebuf)) {

                        switch (i) {
                        case ADF4351_REG1:
                        case ADF4351_REG4:
                                doublebuf = 1;
                                break;
                        }

                        st.val = (st.regs[i] | i);
                        ret = adf4351_write(st.val);
                        if (ret < 0)
                                return ret;
                        st.regs_hw[i] = st.regs[i];
                }
        }
        return 0;
}

/***************************************************************************//**
 * @brief Increases the R counter value until the ADF4351_MAX_FREQ_PFD is
 *        greater than PFD frequency.
 *
 * @param r_cnt - Initial r_cnt value.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
boost::int32_t adf4351_device_t::adf4351_tune_r_cnt(boost::uint16_t r_cnt)
{
        struct adf4351_platform_data *pdata = st.pdata;

        do {
                r_cnt++;
                st.fpfd = (st.clkin * (pdata->ref_doubler_en ? 2 : 1)) /
                           (r_cnt * (pdata->ref_div2_en ? 2 : 1));
        } while (st.fpfd > ADF4351_MAX_FREQ_PFD);

        return r_cnt;
}

/***************************************************************************//**
 * @brief Computes the greatest common divider of two numbers
 *
 * @return Returns the gcd.
*******************************************************************************/
boost::uint32_t adf4351_device_t::gcd(boost::uint32_t x, boost::uint32_t y)
{
        int32_t tmp;
        tmp = y > x ? x : y;
        while((x % tmp) || (y % tmp))
        {
                tmp--;
        }
        return tmp;
}

/***************************************************************************//**
 * @brief Sets the ADF4351 frequency.
 *
 * @param freq - The desired frequency value.
 *
 * @return calculatedFrequency - The actual frequency value that was set.
*******************************************************************************/
boost::int64_t adf4351_device_t::adf4351_set_freq_const_fpfd (boost::uint64_t freq, boost::uint16_t r_cnt)
{
    struct adf4351_platform_data *pdata = st.pdata;
    uint64_t tmp;
    uint32_t prescaler;
    //uint32_t div_gcd;
    //uint16_t mdiv;
    uint8_t band_sel_div;
    int32_t ret;

    if ((freq > ADF4351_MAX_OUT_FREQ) || (freq < ADF4351_MIN_OUT_FREQ))
            return -1;

    st.r4_rf_div_sel = 0;

    while (freq < ADF4351_MIN_VCO_FREQ) {
            freq <<= 1;
            st.r4_rf_div_sel++;
    }

//    if (freq > ADF4351_MAX_FREQ_45_PRESC) {
//            prescaler = ADF4351_REG1_PRESCALER;
//            mdiv = 75;
//    } else {
//            prescaler = 0;
//            mdiv = 23;
//    }
    prescaler = ADF4351_REG1_PRESCALER;

    st.fpfd = (st.clkin * (pdata->ref_doubler_en ? 2 : 1)) /(r_cnt * (pdata->ref_div2_en ? 2 : 1));
    if (st.fpfd > ADF4351_MAX_FREQ_PFD)
        throw pax::runtime_error("[adf4351_device_t] Fpfd out of range!");

    st.r0_int = freq / st.fpfd;
    st.r1_mod = 1000;
    tmp = freq % st.fpfd;
    tmp = tmp * st.r1_mod;
    st.r0_fract = tmp / st.fpfd;
    st.r0_fract = 0;

    band_sel_div = (((st.fpfd) + (ADF4351_MAX_BANDSEL_CLK) - 1) / (ADF4351_MAX_BANDSEL_CLK));	// DIV_ROUND_UP

    if (st.fpfd == ADF4351_MAX_FREQ_PFD)
            band_sel_div = 255;

//    if (st.r0_fract && st.r1_mod) {
//            div_gcd = gcd(st.r1_mod, st.r0_fract);
//            st.r1_mod /= div_gcd;
//            st.r0_fract /= div_gcd;
//    } else {
//            st.r0_fract = 0;
//            st.r1_mod = 1;
//    }

    st.regs[ADF4351_REG0] =
        ADF4351_REG0_INT(st.r0_int) |
        ADF4351_REG0_FRACT(st.r0_fract);

    st.regs[ADF4351_REG1] =
        ADF4351_REG1_PHASE(1) |
        ADF4351_REG1_MOD(st.r1_mod) |
        prescaler;

    st.regs[ADF4351_REG2] =
        ADF4351_REG2_10BIT_R_CNT(r_cnt) |
        ADF4351_REG2_DOUBLE_BUFF_EN |
        (pdata->ref_doubler_en ? ADF4351_REG2_RMULT2_EN : 0) |
        (pdata->ref_div2_en ? ADF4351_REG2_RDIV2_EN : 0) |
        (pdata->r2_user_settings & (ADF4351_REG2_PD_POLARITY_POS |
        ADF4351_REG2_LDP_6ns | ADF4351_REG2_LDF_INT_N |
        ADF4351_REG2_CHARGE_PUMP_CURR_uA(5000) |
        ADF4351_REG2_MUXOUT(0x7) | ADF4351_REG2_NOISE_MODE(0x3)));

    st.regs[ADF4351_REG3] =
        pdata->r3_user_settings &
        (ADF4351_REG3_12BIT_CLKDIV(0xFFF) |
        ADF4351_REG3_12BIT_CLKDIV_MODE(0x3) |
        ADF4351_REG3_12BIT_CSR_EN |
        ADF4351_REG3_CHARGE_CANCELLATION_EN |
        ADF4351_REG3_ANTI_BACKLASH_3ns_EN |
        ADF4351_REG3_BAND_SEL_CLOCK_MODE_HIGH);

    st.regs[ADF4351_REG4] =
            ADF4351_REG4_FEEDBACK_FUND |
            ADF4351_REG4_RF_DIV_SEL(st.r4_rf_div_sel) |
            ADF4351_REG4_8BIT_BAND_SEL_CLKDIV(band_sel_div) |
            ADF4351_REG4_RF_OUT_EN |
            (pdata->r4_user_settings &
            (ADF4351_REG4_OUTPUT_PWR(0x3) |
            ADF4351_REG4_AUX_OUTPUT_PWR(0x3) |
            ADF4351_REG4_AUX_OUTPUT_EN |
            ADF4351_REG4_AUX_OUTPUT_FUND //|
            //ADF4351_REG4_MUTE_TILL_LOCK_EN
             ));

    st.regs[ADF4351_REG5] = ADF4351_REG5_LD_PIN_MODE_DIGITAL + 0x00180000;

    ret = adf4351_sync_config();
    if(ret < 0)
            return ret;

    tmp = (uint64_t)((st.r0_int * st.r1_mod) + st.r0_fract) * (uint64_t)st.fpfd;
    tmp = tmp / ((uint64_t)st.r1_mod * ((uint64_t)1 << st.r4_rf_div_sel));
    std::cout << "ADF4351 internal freq = " << freq << " Fpfd = " << st.fpfd << " RF Div = " << st.r4_rf_div_sel << " N_cnt = " << st.r0_int <<
                  " Frac = " << st.r0_fract  << std::endl;
    std::cout << "ADF4351 set to " << tmp/1e6 << " MHz" << std::endl;
    return tmp;
}

/***************************************************************************//**
 * @brief Sets the ADF4351 frequency.
 *
 * @param freq - The desired frequency value.
 *
 * @return calculatedFrequency - The actual frequency value that was set.
*******************************************************************************/
boost::int64_t adf4351_device_t::adf4351_set_freq(boost::uint64_t freq)
{
    struct adf4351_platform_data *pdata = st.pdata;
    uint64_t tmp;
    uint32_t div_gcd, prescaler, chspc;
    uint16_t mdiv, r_cnt = 0;
    uint8_t band_sel_div;
    int32_t ret;

    if ((freq > ADF4351_MAX_OUT_FREQ) || (freq < ADF4351_MIN_OUT_FREQ))
            return -1;

    st.r4_rf_div_sel = 0;

    while (freq < ADF4351_MIN_VCO_FREQ) {
            freq <<= 1;
            st.r4_rf_div_sel++;
    }

    if (freq > ADF4351_MAX_FREQ_45_PRESC) {
            prescaler = ADF4351_REG1_PRESCALER;
            mdiv = 75;
    } else {
            prescaler = 0;
            mdiv = 23;
    }

    /*
     * Allow a predefined reference division factor
     * if not set, compute our own
     */
    if (pdata->ref_div_factor)
            r_cnt = pdata->ref_div_factor - 1;

    chspc = st.chspc;

    do  {
        do {
            do  {
                r_cnt = adf4351_tune_r_cnt(r_cnt);
                st.r1_mod = st.fpfd / chspc;
                if (r_cnt > ADF4351_MAX_R_CNT) {
                /* try higher spacing values */
                chspc++;
                r_cnt = 0;
                }
            } while ((st.r1_mod > ADF4351_MAX_MODULUS) && r_cnt);
        } while (r_cnt == 0);

        tmp = freq * (uint64_t)st.r1_mod + (st.fpfd > 1);

        tmp = (tmp / st.fpfd);	/* Div round closest (n + d/2)/d */

        st.r0_fract = tmp % st.r1_mod;
        tmp = tmp / st.r1_mod;

        st.r0_int = (uint32_t)tmp;
    } while (mdiv > st.r0_int);

    band_sel_div = (((st.fpfd) + (ADF4351_MAX_BANDSEL_CLK) - 1) / (ADF4351_MAX_BANDSEL_CLK));	// DIV_ROUND_UP

    if (st.fpfd == ADF4351_MAX_FREQ_PFD)
            band_sel_div = 255;

    if (st.r0_fract && st.r1_mod) {
            div_gcd = gcd(st.r1_mod, st.r0_fract);
            st.r1_mod /= div_gcd;
            st.r0_fract /= div_gcd;
    } else {
            st.r0_fract = 0;
            st.r1_mod = 1;
    }

    st.regs[ADF4351_REG0] = ADF4351_REG0_INT(st.r0_int) |
                             ADF4351_REG0_FRACT(st.r0_fract);

    st.regs[ADF4351_REG1] = ADF4351_REG1_PHASE(1) |
                             ADF4351_REG1_MOD(st.r1_mod) |
                             prescaler;

    st.regs[ADF4351_REG2] =
            ADF4351_REG2_10BIT_R_CNT(r_cnt) |
            ADF4351_REG2_DOUBLE_BUFF_EN |
            (pdata->ref_doubler_en ? ADF4351_REG2_RMULT2_EN : 0) |
            (pdata->ref_div2_en ? ADF4351_REG2_RDIV2_EN : 0) |
            (pdata->r2_user_settings & (ADF4351_REG2_PD_POLARITY_POS |
            ADF4351_REG2_LDP_6ns | ADF4351_REG2_LDF_INT_N |
            ADF4351_REG2_CHARGE_PUMP_CURR_uA(5000) |
            ADF4351_REG2_MUXOUT(0x7) | ADF4351_REG2_NOISE_MODE(0x3)));

    st.regs[ADF4351_REG3] = pdata->r3_user_settings &
                             (ADF4351_REG3_12BIT_CLKDIV(0xFFF) |
                             ADF4351_REG3_12BIT_CLKDIV_MODE(0x3) |
                             ADF4351_REG3_12BIT_CSR_EN |
                             ADF4351_REG3_CHARGE_CANCELLATION_EN |
                             ADF4351_REG3_ANTI_BACKLASH_3ns_EN |
                             ADF4351_REG3_BAND_SEL_CLOCK_MODE_HIGH);

    st.regs[ADF4351_REG4] =
            ADF4351_REG4_FEEDBACK_FUND |
            ADF4351_REG4_RF_DIV_SEL(st.r4_rf_div_sel) |
            ADF4351_REG4_8BIT_BAND_SEL_CLKDIV(band_sel_div) |
            ADF4351_REG4_RF_OUT_EN |
            (pdata->r4_user_settings &
            (ADF4351_REG4_OUTPUT_PWR(0x3) |
            ADF4351_REG4_AUX_OUTPUT_PWR(0x3) |
            ADF4351_REG4_AUX_OUTPUT_EN |
            ADF4351_REG4_AUX_OUTPUT_FUND //|
            //ADF4351_REG4_MUTE_TILL_LOCK_EN
             ));

    st.regs[ADF4351_REG5] = ADF4351_REG5_LD_PIN_MODE_DIGITAL + 0x00180000;

    ret = adf4351_sync_config();
    if(ret < 0)
            return ret;

    tmp = (uint64_t)((st.r0_int * st.r1_mod) + st.r0_fract) * (uint64_t)st.fpfd;
    tmp = tmp / ((uint64_t)st.r1_mod * ((uint64_t)1 << st.r4_rf_div_sel));

    return tmp;
}

/***************************************************************************//**
 * @brief Initializes the ADF4351.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
boost::int32_t adf4351_device_t::adf4351_setup()
{
        st.pdata =(adf4351_platform_data*) &adf4351_pdata_lpc;

        set_refin_freq(st.pdata->clkin);
        set_freq_resolution(st.pdata->channel_spacing);

        st.regs[ADF4351_REG5] = //ADF4351_REG5_LD_PIN_MODE_LOW
                ADF4351_REG5_LD_PIN_MODE_DIGITAL + 0x00180000;
        st.regs[ADF4351_REG4] = ADF4351_REG4_FEEDBACK_FUND +
                                                         ADF4351_REG4_8BIT_BAND_SEL_CLKDIV(246) +
                                                         ADF4351_REG4_RF_OUT_EN +
                                                         ADF4351_REG4_OUTPUT_PWR(3);
        st.regs[ADF4351_REG3] = ADF4351_REG3_12BIT_CLKDIV(150);
        st.regs[ADF4351_REG2] = ADF4351_REG2_10BIT_R_CNT(16) +
                             ADF4351_REG2_CHARGE_PUMP_CURR_uA(5000) +
                            //(0xF << 9) +
                             ADF4351_REG2_PD_POLARITY_POS +
                             //ADF4351_REG2_POWER_DOWN_EN;
                            ADF4351_REG2_MUXOUT(0);
        st.regs[ADF4351_REG1] = ADF4351_REG1_PRESCALER +
                             ADF4351_REG1_MOD(1);
        st.regs[ADF4351_REG0] = ADF4351_REG0_FRACT(0) +
                             ADF4351_REG0_INT(1200);

    return adf4351_sync_config();
}

/***************************************************************************//**
 * @brief Stores PLL 0 frequency in Hz.
 *
 * @param Hz - The selected frequency.
 *
 * @return Returns the selected frequency.
*******************************************************************************/
boost::int64_t adf4351_device_t::set_freq(boost::int64_t Hz)
{
    //return adf4351_set_freq(Hz);
    return adf4351_set_freq_const_fpfd(Hz, 16);
}

/***************************************************************************//**
 * @brief Stores PLL 0 frequency resolution/channel spacing in Hz.
 *
 * @param Hz - The selected frequency.
 *
 * @return Returns the selected frequency.
*******************************************************************************/
int32_t adf4351_device_t::set_freq_resolution(int32_t Hz)
{
        if(Hz != INT32_MAX)
        {
                st.chspc = Hz;
        }
        return st.chspc;
}

/***************************************************************************//**
 * @brief Sets PLL 0 REFin frequency in Hz.
 *
 * @param Hz - The selected frequency.
 *
 * @return Returns the selected frequency.
*******************************************************************************/
boost::int64_t adf4351_device_t::set_refin_freq(boost::int64_t Hz){
    if(Hz != INT32_MAX)
    {
            st.clkin = (uint32_t)Hz;
    }

    return st.clkin;
}

/***************************************************************************//**
 * @brief Powers down the PLL.
 *
 * @param pwd - Power option.
 *				Example: 0 - Power up the PLL.
 *						 1 - Power down the PLL.
 *
 * @return Returns the PLL's power status.
*******************************************************************************/
int32_t adf4351_device_t::adf4351_out_altvoltage0_powerdown(int32_t pwd)
{
        if(pwd == 1)
        {
                st.regs[ADF4351_REG2] |= ADF4351_REG2_POWER_DOWN_EN;
                adf4351_sync_config();
        }
        if(pwd == 0)
        {
                st.regs[ADF4351_REG2] &= ~ADF4351_REG2_POWER_DOWN_EN;
                adf4351_sync_config();
        }


        return (st.regs[ADF4351_REG2] & ADF4351_REG2_POWER_DOWN_EN);
}


adf4351::sptr adf4351::make(pax::spi_iface::sptr io_face, uint32_t slave_num)
{
    adf4351_io::sptr io_iface=adf4351_io::make(io_face,slave_num);
    return sptr(new adf4351_device_t(io_iface));
}
