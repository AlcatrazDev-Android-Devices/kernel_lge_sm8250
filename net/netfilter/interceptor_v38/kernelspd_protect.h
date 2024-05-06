/**
   @copyright
   Copyright (c) 2017 - 2018, Rambus Inc. All rights reserved.
*/


#ifndef KERNELSPD_PROTECT_H
#define KERNELSPD_PROTECT_H

#include <linux/skbuff.h>
#include "ip_selector.h"

struct KernelSpdProtectHooks
{
    void (*protect_hook)(
            void *param,
            struct sk_buff *skb);

    void (*initial_fragment_hook)(
            void *param,
            const struct IPSelectorFields *fields,
            uint32_t fragment_id,
            int result);

    int (*other_fragment_hook)(
            void *param,
            const struct IPSelectorFields *fields,
            uint32_t fragment_id,
            struct sk_buff *skb);
};

int
kernelspd_register_protect_hooks(
        struct net *net,
        const struct KernelSpdProtectHooks *hooks,
        void *param);

void
kernelspd_unregister_protect_hooks(
        struct net *net);

int
kernelspd_get_net_id(
        struct net *net);

void
kernelspd_mark_reinject(
        struct sk_buff *skb);

void
kernelspd_skb_get_interfaces(
        struct sk_buff *skb,
        uint32_t *in_interface_index_p,
        uint32_t *out_interface_index_p);

#endif /* KERNELSPD_PROTECT_H */
