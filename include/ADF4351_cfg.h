/***************************************************************************//**
 *   @file   ADF4351_cfg.h
 *   @brief  Header file of ADF4351 Driver Configuration.
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
#ifndef __AD9523_CFG_H__
#define __AD9523_CFG_H__

#include <ADF4351.h>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <pax_iface.h>

struct adf4351_platform_data
{
    uint32_t    clkin;
    uint32_t    channel_spacing;
    uint64_t    power_up_frequency;

    uint16_t        ref_div_factor; /* 10-bit R counter */
    uint8_t		ref_doubler_en;
    uint8_t		ref_div2_en;

    uint32_t	r2_user_settings;
    uint32_t	r3_user_settings;
    uint32_t	r4_user_settings;
    int32_t		gpio_lock_detect;
};




class adf4351_params {
public:
    typedef boost::shared_ptr<adf4351_params> sptr;

    virtual ~adf4351_params() {}

//    virtual digital_interface_delays_t get_digital_interface_timing(size_t witch) = 0;
//    virtual digital_interface_mode_t get_digital_interface_mode() = 0;
//    virtual clocking_mode_t get_clocking_mode() = 0;
//    virtual double get_band_edge(frequency_band_t band) = 0;
//    virtual unsigned int get_switch()=0;
//    virtual void set_switch(unsigned int )=0;
};

class adf4351_ctrl : public boost::noncopyable
{
public:
    typedef boost::shared_ptr<adf4351_ctrl> sptr;

    virtual ~adf4351_ctrl(void) {};

    //! make a new synthesizer control object
    static sptr make_spi(adf4351_params::sptr client_settings, pax_iface::sptr spi_iface, boost::uint32_t slave_num);


};

class adf4351_io
{
public:
    typedef boost::shared_ptr<adf4351_io> sptr;
    static sptr make(pax::spi_iface::sptr iface,boost::uint32_t slave_num);
    virtual void write(boost::uint32_t val) = 0;
};


#endif // __AD9523_CFG_H__
