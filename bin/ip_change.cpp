#include "recorder.hpp"
#include <eeprom.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <log.hpp>


//#define MANUAL_ETH_ADDR_SET 1

int main(int argc, char *argv[])
{


#ifndef MANUAL_ETH_ADDR_SET

    std::string IP;
    std::string MAC;
    std::string IFC;
    std::string RESET_TO_DEFAULT;


    std::string python_str_path(python_script_path); python_str_path+="usrp2_recovery.py";


    boost::program_options::options_description desc("Allowed options:");
    desc.add_options()
            ("help", "Display this help message.")
            ("new-ip", boost::program_options::value<std::string>(&IP), "Specify new IP in this format 192.168.10.3")
            ("new-mac", boost::program_options::value<std::string>(&MAC), "Specify new MAC ADDR in this format 00:50:C2:85:3F:FF")
            ("ifc", boost::program_options::value<std::string>(&IFC), "Specify ethernet interface that pax connected to it(use ipconfig in windows and ifconfig in linux) (only necessary for reset)")
            ("reset", boost::program_options::value<std::string>(&RESET_TO_DEFAULT),"when you burn an invalid ip or mac and pax doesn't respounse you you can use this option to FORCE reset device to default state. value are {true,false}")
            ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if(vm.empty()){
        std::cout<<"use --help for program IP"<<std::endl;
        return EXIT_SUCCESS;
    }

    pax::eth_mac_addr_t mac,mac_echo;
    pax::eth_ip_addr_t ip,ip_echo;

    mac.set_addr();ip.set_addr();

    //Print help message
    if(vm.count("help") > 0){
         boost::format("PAX ETHERNET IP CHANGER\n");
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }
	if (vm.count("reset") > 0) {
#ifndef WINDOWS

		if (RESET_TO_DEFAULT == "true") {
			if (vm.count("ifc") == 0) {
				std::cerr << "please specify ifc" << std::endl;
				return EXIT_FAILURE;
			}

//			std::cout << "TEST DUDE:" << (std::string)(python_str_path) << std::endl;

            std::string cmd = ("sudo python " + python_str_path + " --ifc " + IFC + " --new-ip " + ip.get_addr() + " \n");
			system(cmd.c_str());
			return EXIT_SUCCESS;
		}
		else if (RESET_TO_DEFAULT == "false") {
		}
		else {
			std::cerr << "WRONG COMMAND FOR reset-to-default" << std::endl;
			return EXIT_FAILURE;
		}
#else
		std::cout << "NOT IMPLEMENTED FOR WINDOWS USER" << std::endl;
		return EXIT_FAILURE;
#endif
	}



    bool mac_change=false,ip_change=false;
    try{
        if(vm.count("new-ip") > 0){
            ip_change = true;
            ip.set_addr(IP);
        }if(vm.count("new-mac") > 0){
            mac_change=true;
            mac.set_addr(MAC);
        }
    }catch(pax::value_error &e){
        std::cerr << e.what() <<std::endl;
        return EXIT_FAILURE;
    }


    mb_container_type tester;
    pax_init(tester,0);

    pax::eeprom::sptr db=pax::eeprom::make(tester.iface);



    ///////////////////////////////////////////////////////
    //SET IP AND MAC                                    //
    //////////////////////////////////////////////////////


    if(ip_change)
        db->set_ip_addr(ip);
    if(mac_change)
        db->set_mac_addr(mac);


    //////////////////////////////////////////////////////
    //READ BACK AND CHECK VALIDATION                    //
    /////////////////////////////////////////////////////
    if(ip_change){
        ip_echo = db->get_ip_addr();
        std::cout<<"readback ip  is = "<<ip_echo.get_addr()<<std::endl;
        std::cout<<"expected ip  is = "<<ip.get_addr()<<std::endl;
    }

    std::cout<<std::endl;

    if(mac_change){
        mac_echo = db->get_mac_addr();
        std::cout<<"readback mac is = "<<mac_echo.get_addr()<<std::endl;
        std::cout<<"expected mac is = "<<mac.get_addr()<<std::endl;
    }




    if( (!(ip == ip_echo) && ip_change) || (!(mac == mac_echo) && mac_change)){
        std::cerr<<"IP or MAC can not changed be changed!"<<std::endl
                 <<"this may have multiple reason"<<std::endl
                 <<"1- EEPROM NOT work correctly"<<std::endl
                 <<"2- daughter board is NOT connected"<<std::endl
                 <<"3- WRONG slave addresing of I2C for EEprom please check fw_common.h global variable USRP2_I2C_DEV_EEPROM"<<std::endl;

        std::string error="(";
        if(!(ip == ip_echo) && ip_change){
            error += " IP ";
        }
        if(!(mac == mac_echo) && mac_change){
            error += " MAC ";
        }
        error += ") changed process failed";
        throw pax::io_error(error.c_str());

    } else {
        std::cout<<"NOTE : ip change is done successfully please reset the fpga to relad new IP and MAC"<<std::endl;
    }




#else

    eth_mac_addr_t mac,mac_echo;
    eth_ip_addr_t ip,ip_echo;
    mb_container_type tester;
    pax_init(tester,2);

    eeprom::sptr db=eeprom::make(tester.iface);



    ///////////////////////////////////////////////////////
    //SET IP AND MAC                                    //
    //////////////////////////////////////////////////////


    mac.set_addr("00:50:C2:85:3F:FF");ip.set_addr("192.168.10.3");

    db->set_ip_addr(ip);
    db->set_mac_addr(mac);


    //////////////////////////////////////////////////////
    //READ BACK AND CHECK VALIDATION                    //
    /////////////////////////////////////////////////////

    ip_echo = db->get_ip_addr();
    mac_echo = db->get_mac_addr();
    std::cout<<"readback ip  is = "<<ip_echo.get_addr()<<std::endl;
    std::cout<<"expected ip  is = "<<ip.get_addr()<<std::endl;
    std::cout<<std::endl;
    std::cout<<"readback mac is = "<<mac_echo.get_addr()<<std::endl;
    std::cout<<"expected mac is = "<<mac.get_addr()<<std::endl;


    if( !(ip == ip_echo) || !(mac == mac_echo)){
        std::cerr<<"IP or MAC can not changed be changed!"<<std::endl
                 <<"this may have multiple reason"<<std::endl
                 <<"1- EEPROM NOT work correctly"<<std::endl
                 <<"2- daughter board is NOT connected"<<std::endl
                 <<"3- WRONG slave addresing of I2C for EEprom please check fw_common.h global variable USRP2_I2C_DEV_EEPROM"<<std::endl;
    }

    if(!(ip == ip_echo)){
        throw io_error("IP changed process failed");
    }
    if(!(mac == mac_echo)){
        throw io_error("MAC changed process failed");
    }
//    std::string cmd=("sudo "+python_str_path+" --ifc "+IFC+" --new-ip "+ip.get_addr()+" \n");
//    system(cmd.c_str());

#endif

    return 0;
}

