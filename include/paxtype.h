#ifndef PAXTYPE_H
#define PAXTYPE_H



struct mtu_result_t{
    size_t recv_mtu, send_mtu;
};

  mtu_result_t determine_mtu(const std::string &addr, const mtu_result_t &user_mtu);
#endif // PAXTYPE_H

