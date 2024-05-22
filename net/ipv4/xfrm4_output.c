/*
 * xfrm4_output.c - Common IPsec encapsulation code for IPv4.
 * Copyright (c) 2004 Herbert Xu <herbert@gondor.apana.org.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/netfilter_ipv4.h>
#include <net/dst.h>
#include <net/ip.h>
#include <net/xfrm.h>
#include <net/icmp.h>

#include <net/patchcodeid.h>

static int xfrm4_tunnel_check_size(struct sk_buff *skb)
{
	int mtu, ret = 0;

	if (IPCB(skb)->flags & IPSKB_XFRM_TUNNEL_SIZE)
		goto out;

	if (!(ip_hdr(skb)->frag_off & htons(IP_DF)) || skb->ignore_df)
		goto out;

	mtu = dst_mtu(skb_dst(skb));
	if ((!skb_is_gso(skb) && skb->len > mtu) ||
	    (skb_is_gso(skb) &&
	     !skb_gso_validate_network_len(skb, ip_skb_dst_mtu(skb->sk, skb)))) {
		skb->protocol = htons(ETH_P_IP);

		if (skb->sk)
			xfrm_local_error(skb, mtu);
		else
			icmp_send(skb, ICMP_DEST_UNREACH,
				  ICMP_FRAG_NEEDED, htonl(mtu));
		ret = -EMSGSIZE;
	}
out:
	return ret;
}

int xfrm4_extract_output(struct xfrm_state *x, struct sk_buff *skb)
{
	int err;

	err = xfrm4_tunnel_check_size(skb);
	if (err)
		return err;

	XFRM_MODE_SKB_CB(skb)->protocol = ip_hdr(skb)->protocol;

	return xfrm4_extract_header(skb);
}

int xfrm4_prepare_output(struct xfrm_state *x, struct sk_buff *skb)
{
	int err;

	err = xfrm_inner_extract_output(x, skb);
	if (err)
		return err;

	IPCB(skb)->flags |= IPSKB_XFRM_TUNNEL_SIZE;
	skb->protocol = htons(ETH_P_IP);

	return x->outer_mode->output2(x, skb);
}
EXPORT_SYMBOL(xfrm4_prepare_output);

int xfrm4_output_finish(struct sock *sk, struct sk_buff *skb)
{
	memset(IPCB(skb), 0, sizeof(*IPCB(skb)));

	IPCB(skb)->flags |= IPSKB_XFRM_TRANSFORMED;

	return xfrm_output(sk, skb);
}

/* 2018-03-16 gihong.jang@lge.com LGP_DATA_KERNEL_XFRM_FRAG_ESP [START]*/
#ifdef CONFIG_XFRM_FRAG_ESP_BEFORE_TUNNEL_ENC
static int __xfrm4_output_finish(struct net *net, struct sock *sk, struct sk_buff *skb)
{
	struct xfrm_state *x = skb_dst(skb)->xfrm;
	return x->outer_mode->afinfo->output_finish(sk, skb);
}
#endif
/* 2018-03-16 gihong.jang@lge.com LGP_DATA_KERNEL_XFRM_FRAG_ESP [END]*/
static int __xfrm4_output(struct net *net, struct sock *sk, struct sk_buff *skb)
{
	struct xfrm_state *x = skb_dst(skb)->xfrm;

#ifdef CONFIG_NETFILTER
	if (!x) {
		IPCB(skb)->flags |= IPSKB_REROUTED;
		return dst_output(net, sk, skb);
	}
#endif

/* 2018-03-16 gihong.jang@lge.com LGP_DATA_KERNEL_XFRM_FRAG_ESP [START]*/
#ifdef CONFIG_XFRM_FRAG_ESP_BEFORE_TUNNEL_ENC
	patch_code_id("LPCP-2381@n@c@vmlinux@xfrm4_output.c@1");
	if (x->props.mode == XFRM_MODE_TUNNEL &&
			skb != NULL && skb->protocol == htons(ETH_P_IP) &&
			skb->sk != NULL && skb->sk->sk_protocol != IPPROTO_TCP &&
			skb_dst(skb)->next != NULL && skb_dst(skb)->next->xfrm != NULL &&
			!(skb_dst(skb)->next->xfrm->outer_mode->flags & XFRM_MODE_FLAG_TUNNEL)) {
		int mtu;
		bool toobig;

		if (skb->protocol == htons(ETH_P_IP)) {
			mtu = ip_skb_dst_mtu(sk,skb);
		} else {
			mtu = dst_mtu(skb_dst(skb));
		}

		toobig = skb->len > mtu && !skb_is_gso(skb);

		if (toobig && (ip_hdr(skb)->frag_off & htons(IP_DF))) {
			xfrm_local_error(skb, mtu);
			return -EMSGSIZE;
		} else if (!skb->ignore_df && toobig && skb->sk) {
			xfrm_local_error(skb, mtu);
			return -EMSGSIZE;
		}

		if (toobig || dst_allfrag(skb_dst(skb))) {
			printk("XFRM_FRAG_ESP_BEFORE_TUNNEL_ENC go fragment\n");
			return ip_do_fragment(net, sk, skb, __xfrm4_output_finish);
		}
	}
#endif
/* 2018-03-16 gihong.jang@lge.com LGP_DATA_KERNEL_XFRM_FRAG_ESP [END]*/

	return x->outer_mode->afinfo->output_finish(sk, skb);
}

int xfrm4_output(struct net *net, struct sock *sk, struct sk_buff *skb)
{
	return NF_HOOK_COND(NFPROTO_IPV4, NF_INET_POST_ROUTING,
			    net, sk, skb, NULL, skb_dst(skb)->dev,
			    __xfrm4_output,
			    !(IPCB(skb)->flags & IPSKB_REROUTED));
}

void xfrm4_local_error(struct sk_buff *skb, u32 mtu)
{
	struct iphdr *hdr;

	hdr = skb->encapsulation ? inner_ip_hdr(skb) : ip_hdr(skb);
	ip_local_error(skb->sk, EMSGSIZE, hdr->daddr,
		       inet_sk(skb->sk)->inet_dport, mtu);
}
