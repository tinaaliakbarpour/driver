    #include "recorder.hpp"
#include <eeprom.hpp>

#define LEFTROTATE(x, c) (((x) << (c)) | ((x) >> (32 - (c))))
// These vars will contain the hash
unsigned int h0, h1, h2, h3;
void md5(unsigned char *initial_msg, unsigned int initial_len);



int main(int argc, char* argv[])
{
    (void)argc;
    (void) argv;

    int a = 1;
    a = a<<16;

boost::cnv::cstream ccnv;

    mb_container_type tester;
    pax_init(tester,0,true);

    pax::eth_mac_addr_t mac;
    pax::eth_ip_addr_t ip;



    mac.set_addr("00:50:C2:85:3F:FF");ip.set_addr("192.168.10.3");
    std::string board_type("8K410T-rev.1.0");       // "PAX2.D-rev.0.1"   "PAXGNS-rev.0.0"
                                                    // "PAX2S6-rev.0.0" "PAX2S6-rev.1.0"
                                                    // "8K410T-rev.1.0" "PAX8V7-rev.1.0"



    std::string serial_daughter("A20100");    //512020               // 057058 /////6dig
    std::string serial_mother=(std::string("S/N : ")+"A20100"); //A095011 /////6dig


    pax::eeprom::sptr db=pax::eeprom::make(tester.iface);

    db->set_ip_addr(ip);
    db->set_mac_addr(mac);

    pax::eeprom::board_info info;
    info.board_mfg.assign("PA Ltd.");//"PartoCom Asia Ltd.");            //config
    info.part.assign("AD9364");                             //config
    info.product_name.assign("PAXGNS8 Monitorin");          //"PAX2S6 Analog Kit"  "PAXGNS8 Monitorin"
                                                            //"PAXGNS Analog Kit"  "PAXGNS Analog CAL"
                                                            //"PAX2K7 FILTERBANK"  "PAX8V7 FILTERBANK"

    info.ad_ref_clk.assign("20");                           // 40 20 ----default : 40---- just for sim

    info.serial.assign(serial_daughter);                     //config
    db->set_info(info);
    db->write_info();


/*    if (tester.iface->get_flash_type()==pax::flash::SPI_IS25LPxxx){

        std::vector<uint8_t> pp(8,0);

        pax::flash_vec_t temp;
        temp=tester.iface->read_uid_flash();

        for(uint8_t i=0;i<8;i++){
            pp[i]=(temp[2*i]>>1)+(temp[2*i+1]>>1);
        }

        md5(pp.data(),8);

        temp.resize(16);

        temp[0x0]=(h0>>24) & 0xff;
        temp[0x1]=(h0>>16) & 0xff;
        temp[0x2]=(h0>>8) & 0xff;
        temp[0x3]=(h0>>0) & 0xff;

        temp[0x4]=(h1>>24) & 0xff;
        temp[0x5]=(h1>>16) & 0xff;
        temp[0x6]=(h1>>8) & 0xff;
        temp[0x7]=(h1>>0) & 0xff;

        temp[0x8]=(h2>>24) & 0xff;
        temp[0x9]=(h2>>16) & 0xff;
        temp[0xa]=(h2>>8) & 0xff;
        temp[0xb]=(h2>>0) & 0xff;

        temp[0xc]=(h3>>24) & 0xff;
        temp[0xd]=(h3>>16) & 0xff;
        temp[0xe]=(h3>>8) & 0xff;
        temp[0xf]=(h3>>0) & 0xff;

    //    // WARNING: Be careful with this!
        tester.iface->erase_otp_flash(pax::flash::IS25LPxx_OTP_REG_0x300_TO_0x3ff);
        tester.iface->write_otp_flash(0,temp,pax::flash::IS25LPxx_OTP_REG_0x300_TO_0x3ff);
        tester.iface->lock_otp_flash(pax::flash::IS25LPxx_OTP_REG_0x300_TO_0x3ff);

        temp.clear();
        std::string mb_data= board_type+"  "+serial_mother;
        for(uint32_t i=0;i<mb_data.length();i++)
            temp.push_back(mb_data[i]);

        tester.iface->erase_otp_flash(pax::flash::IS25LPxx_OTP_REG_0x000_TO_0x0ff);
        tester.iface->write_otp_flash(0,temp,pax::flash::IS25LPxx_OTP_REG_0x000_TO_0x0ff);
        tester.iface->lock_otp_flash(pax::flash::IS25LPxx_OTP_REG_0x000_TO_0x0ff);



    }  else if (tester.iface->get_flash_type()==pax::flash::BPI_S29GL01GS) {
        pax::flash_vec_t uid = tester.iface->read_uid_flash();//PH
        pax::uint8_vector_t to_1_byte(5,0);
        for(uint8_t i=0;i<5;i++)
        {
            to_1_byte[2*i]=uid[i]>>8;
            to_1_byte[2*i+1]=uid[i]&0xff;
        }

        md5(to_1_byte.data(),10);

        pax::flash_vec_t password(8,0);

        password[0]=h0>>16;
        password[1]=h0&0xffff;
        password[2]=h1>>16;
        password[3]=h1&0xffff;
        password[4]=h2>>16;
        password[5]=h2&0xffff;
        password[6]=h3>>16;
        password[7]=h3&0xffff;

        pax::flash_vec_t write_to_flash;

        pax::flash_vec_t board_type_(7,0);
        memcpy(board_type_.data(),board_type.c_str(),14);  //config
        board_type_.push_back(0xFFFF);

        for(uint8_t i=0;i<board_type_.size();i++)
            write_to_flash.push_back(board_type_[i]);

        pax::flash_vec_t serial_mother_(6,0);
        memcpy(serial_mother_.data(),serial_mother.c_str(),12);                 //config
        serial_mother_.push_back(0xFFFF);

        for(uint8_t i=0;i<serial_mother_.size();i++)
            write_to_flash.push_back(serial_mother_[i]);

        for(uint8_t i=0;i<password.size();i++)
            write_to_flash.push_back(password[i]);

        tester.iface->write_otp_flash(0,write_to_flash,pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);//PH
        tester.iface->lock_otp_flash(pax::flash::BPI_S29GL01GS_256_WORD_OTP_REG_SEC_1);//PH

    } else if(tester.iface->get_flash_type()==pax::flash::BPI_28F00AP30){
        std::vector<uint8_t> pp(80,0);
        std::vector<uint16_t> p(40,0);

        tester.iface->write_flash(0x81,0x90);

        for(int i=0;i<40;i++)
        {
            p[i]=(uint16_t)tester.iface->read_flash(0x81+i);
            pp[2*i]=p[i]>>8;
            pp[2*i+1]=p[i]&0xff;
        }

        md5(pp.data(),8);

        p.resize(32);

        p[0]=h0>>16;
        p[1]=h0&0xffff;

        p[2]=h1>>16;
        p[3]=h1&0xffff;

        p[4]=h2>>16;
        p[5]=h2&0xffff;

        p[6]=h3>>16;
        p[7]=h3&0xffff;


    //    // WARNING: Be careful with this!
        for(int i=0; i<8;i++){
            tester.iface->write_flash(0x8A+i,0xC0);  // OTP Program Setup
            tester.iface->write_flash(0x8A+i,p[i]); // Confirm Data

            boost::uint32_t command_complete=0;
            boost::uint32_t  status;

            while (!command_complete){
                status = tester.iface->read_flash(0); // does address matter????
                command_complete=!!(status&0x80);  // check ready status
            }
        }
        memcpy(p.data(),board_type.c_str(),14);  //config
        for(int i=0; i<7;i++){
            tester.iface->write_flash(0x92+i,0xC0);  // OTP Program Setup
            tester.iface->write_flash(0x92+i,p[i]); // Confirm Data

            boost::uint32_t command_complete=0;
            boost::uint32_t  status;

            while (!command_complete){
                status = tester.iface->read_flash(0); // does address matter????
                command_complete=!!(status&0x80);  // check ready status
            }
        }
        memcpy(p.data(),serial_mother.c_str(),12);                 //config
        for(int i=0; i<6;i++){
            tester.iface->write_flash(0x9A+i,0xC0);  // OTP Program Setup
            tester.iface->write_flash(0x9A+i,p[i]); // Confirm Data

            boost::uint32_t command_complete=0;
            boost::uint32_t  status;

            while (!command_complete){
                status = tester.iface->read_flash(0); // does address matter????
                command_complete=!!(status&0x80);  // check ready status
            }
        }

        tester.iface->write_flash(0x89,0xC0);  // OTP Program Setup
        tester.iface->write_flash(0x89,0xFFF8); // Confirm Data
    }


*/
    return 0;
}




template<typename tp>
void byteswap(std::vector<tp>& in)
{
    for(int i=0;i<in.size();i++)
        in[i]=pax::byteswap(in[i]);
}



void md5(unsigned char *initial_msg, unsigned int initial_len) {

    // Message (to prepare)
    unsigned char msg[120];
    unsigned int i;

    // Note: All variables are unsigned 32 bit and wrap modulo 2^32 when calculating

    // r specifies the per-round shift amounts
    unsigned int r[] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                        5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
                        4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                        6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

    // Use binary integer part of the sines of integers (in radians) as constants// Initialize variables:
    unsigned int k[] = {
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
        0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
        0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
        0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
        0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
        0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
        0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
        0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
        0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
        0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
        0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
        0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
        0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
        0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
        0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};

    h0 = 0x67452301;
    h1 = 0xefcdab89;
    h2 = 0x98bcdcfe;
    h3 = 0x10325476;

    // Pre-processing: adding a single 1 bit
    //append "1" bit to message
    /* Notice: the input bytes are considered as bits strings,
       where the first bit is the most significant bit of the byte.[37] */

    // Pre-processing: padding with zeros
    //append "0" bit until message length in bit ≡ 448 (mod 512)
    //append length mod (2 pow 64) to message

    int new_len=448;
    // for(new_len = initial_len*8 + 1; new_len%512!=448; new_len++);
    new_len /= 8;

    //msg = (u_int8_t*)calloc(new_len + 64, 1); // also appends "0" bits
    // (we alloc also 64 extra bytes...)

    for(i=0;i<initial_len;i++)
        msg[i]=initial_msg[i];
    //memcpy(msg, initial_msg, initial_len);
    msg[initial_len] = 128; // write the "1" bit

    for(i=initial_len+1;i<120;i++) // write "0" bits
        msg[i]=0;

    unsigned int bits_len = 8*initial_len; // note, we append the len

    *((int*)(msg+new_len)) = pax::byteswap(bits_len);
    //memcpy(msg + new_len, &bits_len, 4);           // in bits at the end of the buffer

    // Process the message in successive 512-bit chunks:
    //for each 512-bit chunk of message:
    int offset=0;

    unsigned int *w = (unsigned int *) (msg + offset);
    unsigned int a = h0;
    unsigned int b = h1;
    unsigned int c = h2;
    unsigned int d = h3;

    // Main loop:
    for(i = 0; i<64; i++) {
        unsigned int f, g;

        if (i < 16) {
            f = (b & c) | ((~b) & d);
            g = i;
        } else if (i < 32) {
            f = (d & b) | ((~d) & c);
            g = (5*i + 1) % 16;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3*i + 5) % 16;
        } else {
            f = c ^ (b | (~d));
            g = (7*i) % 16;
        }

        unsigned int temp = d;
        d = c;
        c = b;
        unsigned int W=pax::byteswap(w[g]);
        b = b + LEFTROTATE((a + f + k[i] + W ), r[i]);
        a = temp;
    }

    h0 += a;
    h1 += b;
    h2 += c;
    h3 += d;
}
