#include <net/sock.h>

void tcp_enter_quickack_mode(struct sock *sk, unsigned int max_quickacks);