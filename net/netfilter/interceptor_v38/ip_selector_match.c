/**
   @copyright
   Copyright (c) 2013 - 2020, Rambus Inc. All rights reserved.
*/

#include "implementation_defs.h"
#include "ip_selector_match.h"

#define __DEBUG_MODULE__ ipselector

int
ip_selector_match_validate_selector(
        struct IPSelector *selector,
        uint32_t length)
{
    int size = sizeof *selector;

    if (length < size)
    {
        DEBUG_FAIL(
                "Selector %p check failed length %d shorter "
                "than size of header %d.",
                selector,
                length,
                (int) sizeof *selector);

        return -1;
    }

    size +=
        selector->source_endpoint_count *
        sizeof (struct IPSelectorEndpoint);

    size +=
        selector->destination_endpoint_count *
        sizeof (struct IPSelectorEndpoint);

    size +=
        selector->source_address_count *
        sizeof (struct IPSelectorAddress);

    size +=
        selector->destination_address_count *
        sizeof (struct IPSelectorAddress);

    size += selector->source_port_count *
        sizeof (struct IPSelectorPort);

    size += selector->destination_port_count *
        sizeof (struct IPSelectorPort);

    size += selector->interface_bytecount;

    if (length < size)
    {
        DEBUG_FAIL(
                "Selector %p check failed length %u shorter "
                "than computed size %d.",
                selector,
                length,
                size);

        return -1;
    }

    if (selector->bytecount != size)
    {
        DEBUG_FAIL(
                "Selector %p check failed bytecount %d not "
                "equal to computed size %d.",
                selector,
                selector->bytecount,
                size);

        return -1;
    }

    return size;
}


int
ip_selector_match_validate_selector_group(
        const struct IPSelectorGroup *selector_group,
        unsigned bytecount)
{
    unsigned selector_count;

    if (bytecount < sizeof *selector_group)
    {
        DEBUG_FAIL(
                "IP Selector Group header size %d "
                "is bigger than given size %d.",
                (int) sizeof *selector_group,
                (int) bytecount);

        return -1;
    }

    if (bytecount != selector_group->bytecount)
    {
        DEBUG_FAIL(
                "IP Selector Group bytecount %d not equal to given size %d.",
                (int) selector_group->bytecount,
                (int) bytecount);

        return -1;
    }

    selector_count = selector_group->selector_count;

    if (bytecount / sizeof(struct IPSelector) < selector_count)
    {
        DEBUG_FAIL(
                "IP Selector Group selector count %u too big "
                "for given size %d.",
                selector_count,
                (int) bytecount);

        return -1;
    }

    {
        int i;
        char *start = (void *) selector_group;
        unsigned offset = sizeof *selector_group;
        int status = 0;


        for (i = 0; i < selector_count; i++)
        {
            status =
                ip_selector_match_validate_selector(
                        (struct IPSelector *) (start + offset),
                        bytecount - offset);
            if (status < 0)
            {
                return -1;
            }

            offset += status;
        }

        if (offset != bytecount)
        {
            DEBUG_FAIL(
                    "IP Selector Group %p check failed computed size %d "
                    "differs from length %d.",
                    selector_group,
                    offset,
                    bytecount);

            return -1;
        }
    }

    return 0;
}


static int
ip_selector_match_debug_log_endpoint_list(
        const uint8_t *base,
        unsigned offset,
        int endpoint_count)
{
    int i;

    for (i = 0; i < endpoint_count; i++)
    {
        struct IPSelectorEndpoint *endpoint = (void *) (base + offset);

        DEBUG_LOW(
                "Selector %p: "
                "ip_version %d, ip_protocol %d, %pI6c - %pI6c, %d - %d.",
                base,
                endpoint->ip_version,
                endpoint->ip_protocol,
                endpoint->address.begin,
                endpoint->address.end,
                endpoint->port.begin,
                endpoint->port.end);

        offset += sizeof *endpoint;
    }

    return offset;
}


static int
ip_selector_match_debug_log_address_list(
        const uint8_t *base,
        unsigned offset,
        int address_count)
{
    int i;

    for (i = 0; i < address_count; i++)
    {
        struct IPSelectorAddress *address = (void *) (base + offset);

        DEBUG_LOW(
                "Selector %p: %pI6c - %pI6c.",
                base,
                address->begin,
                address->end);

        offset += sizeof *address;
    }

    return offset;
}


static int
ip_selector_match_debug_log_port_list(
        const uint8_t *base,
        unsigned offset,
        int port_count)
{
    int i;

    for (i = 0; i < port_count; i++)
    {
        struct IPSelectorPort *port = (void *) (base + offset);

        DEBUG_LOW(
                "Selector %p: %d - %d.",
                base,
                port->begin,
                port->end);

        offset += sizeof *port;
    }

    return offset;
}


static int
ip_selector_match_debug_log_interface(
        const uint8_t *base,
        unsigned offset,
        int interface_bytecount)
{
    if (interface_bytecount > 0)
    {
        DEBUG_LOW(
                "Selector %p: interface %s.",
                base,
                (char *) (base + offset));

        offset += interface_bytecount;
    }

    return offset;
}


void
ip_selector_match_debug_log_selector(
        const struct IPSelector *selector)
{
    const uint8_t *base = (void *) selector;
    unsigned offset = sizeof *selector;

    DEBUG_LOW(
            "Selector %p: ip version %d.",
            selector,
            selector->ip_version);

    DEBUG_LOW(
            "Selector %p: ip protocol %d.",
            selector,
            selector->ip_protocol);

    DEBUG_LOW(
            "Selector %p: Source endpoint count %d.",
            selector,
            selector->source_endpoint_count);

    offset =
        ip_selector_match_debug_log_endpoint_list(
                base,
                offset,
                selector->source_endpoint_count);

    DEBUG_LOW(
            "Selector %p: Destination endpoint count %d.",
            selector,
            selector->destination_endpoint_count);

    offset =
        ip_selector_match_debug_log_endpoint_list(
                base,
                offset,
                selector->destination_endpoint_count);

    DEBUG_LOW(
            "Selector %p: Source address count %d.",
            selector,
            selector->source_address_count);

    offset =
        ip_selector_match_debug_log_address_list(
                 base,
                 offset,
                 selector->source_address_count);

    DEBUG_LOW(
            "Selector %p: Destination address count %d.",
            selector,
            selector->destination_address_count);

    offset =
        ip_selector_match_debug_log_address_list(
                 base,
                 offset,
                 selector->destination_address_count);

    DEBUG_LOW(
            "Selector %p: Source port count %d.",
            selector,
            selector->source_port_count);

    offset =
        ip_selector_match_debug_log_port_list(
                 base,
                 offset,
                 selector->source_port_count);

    DEBUG_LOW(
            "Selector %p: Destination port count %d.",
            selector,
            selector->destination_port_count);

    offset =
        ip_selector_match_debug_log_port_list(
                 base,
                 offset,
                 selector->destination_port_count);

    DEBUG_LOW(
            "Selector %p: Interface byte count %d.",
            selector,
            selector->interface_bytecount);

    (void)
        ip_selector_match_debug_log_interface(
                 base,
                 offset,
                 selector->interface_bytecount);
}


void
ip_selector_match_debug_log_group(
        const struct IPSelectorGroup *selector_group)
{
    DEBUG_LOW(
            "Selector_Group %p: selector count %u, bytecount %u:",
            selector_group,
            selector_group->selector_count,
            selector_group->bytecount);

    {
        const uint8_t *base = (void *) selector_group;
        unsigned offset = sizeof *selector_group;
        int i;

        for (i = 0; i < selector_group->selector_count; i++)
        {
            const struct IPSelector *selector = (void *)(base + offset);

            DEBUG_LOW(
                    "Selector_Group %p: Selector %p:",
                    selector_group,
                    selector);

            ip_selector_match_debug_log_selector(selector);

            offset += selector->bytecount;
        }
    }
}


const char *
debug_str_ip_selector_fields(
        void *buf,
        const void *data,
        int len)
{
    const struct IPSelectorFields *fields = data;
    char *p;
    int p_len;
    unsigned offset = 0;

    ASSERT(len == sizeof *fields);

    DEBUG_STRBUF_BUFFER_GET(buf, &p, &p_len);

    offset +=
        snprintf(
                p + offset,
                p_len,
                "IPv%d proto %d [",
                fields->ip_version,
                fields->ip_protocol);

    offset +=
        format_ipaddress(
                p + offset, p_len - offset,
                fields->source_address);

    if (fields->ip_protocol == /* ICMP */ 1 ||
        fields->ip_protocol == /* ICMPv6 */ 58 ||
        fields->source_port < 0)
    {
        offset +=
            snprintf(p + offset,
                     p_len - offset,
                     "] -> [");
    }
    else
    {
        offset +=
            snprintf(p + offset,
                     p_len - offset,
                     "]:%d -> [",
                     fields->source_port);
    }

    offset +=
        format_ipaddress(
                p + offset, p_len - offset,
                fields->destination_address);

    if (fields->ip_protocol == /* ICMP */ 1 ||
        fields->ip_protocol == /* ICMPv6 */ 58)
    {
        offset +=
            snprintf(p + offset,
                     p_len - offset,
                     "] type %d code %d",
                     (int) ((fields->source_port >> 8) & 0xff),
                     (int) ((fields->source_port) & 0xff));
    }
    else
    if (fields->destination_port < 0)
    {
        offset +=
            snprintf(p + offset,
                     p_len - offset,
                     "]");
    }
    else
    {
        offset +=
            snprintf(p + offset,
                     p_len - offset,
                     "]:%d",
                     fields->destination_port);
    }

    DEBUG_STRBUF_BUFFER_COMMIT(buf, offset + 1);

    return p;
}


static int
debug_dump_endpoint_list(
        void *context,
        const char *prefix,
        const uint8_t *base,
        unsigned offset,
        unsigned bytecount,
        int endpoint_count)
{
    int i;

    for (i = 0; i < endpoint_count && offset < bytecount; i++)
    {
        struct IPSelectorEndpoint *endpoint = (void *) (base + offset);

        offset += sizeof *endpoint;
        if (offset <= bytecount)
        {
            DEBUG_DUMP_LINE(
                    context,
                    "  %sendpoint ipv %d, proto %d, [%s - %s], [%d - %d].",
                    prefix,
                    endpoint->ip_version,
                    endpoint->ip_protocol,
                    debug_strbuf_ipaddress(
                            DEBUG_STRBUF_GET(),
                            endpoint->address.begin),
                    debug_strbuf_ipaddress(
                            DEBUG_STRBUF_GET(),
                            endpoint->address.end),
                    endpoint->port.begin,
                    endpoint->port.end);
        }
        else
        {
            DEBUG_DUMP_LINE(
                    context,
                    "offset %d exceeded byte count %d",
                    offset,
                    bytecount);
        }
    }

    return offset;
}


static int
debug_dump_address_list(
        void *context,
        const char *prefix,
        const uint8_t *base,
        unsigned offset,
        unsigned bytecount,
        int address_count)
{
    int i;

    for (i = 0; i < address_count && offset < bytecount; i++)
    {
        struct IPSelectorAddress *address = (void *) (base + offset);

        offset += sizeof *address;
        if (offset <= bytecount)
        {
            DEBUG_DUMP_LINE(
                    context,
                    "  %saddress range [%s - %s]",
                    prefix,
                    debug_strbuf_ipaddress(DEBUG_STRBUF_GET(), address->begin),
                    debug_strbuf_ipaddress(DEBUG_STRBUF_GET(), address->end));
        }
        else
        {
            DEBUG_DUMP_LINE(
                    context,
                    "offset %d exceeded byte count %d",
                    offset,
                    bytecount);
        }
    }

    return offset;
}


static int
debug_dump_port_list(
        void *context,
        const char *prefix,
        const uint8_t *base,
        unsigned offset,
        unsigned bytecount,
        int port_count)
{
    int i;

    for (i = 0; i < port_count && offset < bytecount; i++)
    {
        struct IPSelectorPort *port = (void *) (base + offset);

        offset += sizeof *port;
        if (offset <= bytecount)
        {
            DEBUG_DUMP_LINE(
                    context,
                    "  %sport range [%d-%d]",
                    prefix,
                    port->begin,
                    port->end);
        }
        else
        {
            DEBUG_DUMP_LINE(
                    context,
                    "offset %d exceeded byte count %d",
                    offset,
                    bytecount);
        }
    }

    return offset;
}


void
debug_dump_ip_selector(
        void *context,
        const void *data,
        unsigned bytecount)
{
    const struct IPSelector *selector = data;
    const uint8_t *base = (void *) data;
    unsigned offset = sizeof *selector;

    DEBUG_DUMP_LINE(
            context,
            "ipv %d proto %d sec %d dec %d sac %d dac %d spc %d dpc %d:",
            selector->ip_version,
            selector->ip_protocol,
            selector->source_endpoint_count,
            selector->destination_endpoint_count,
            selector->source_address_count,
            selector->destination_address_count,
            selector->source_port_count,
            selector->destination_port_count);

    offset =
        debug_dump_endpoint_list(
                context,
                "src ",
                base,
                offset,
                bytecount,
                selector->source_endpoint_count);

    offset =
        debug_dump_endpoint_list(
                context,
                "dst ",
                base,
                offset,
                bytecount,
                selector->destination_endpoint_count);

    offset =
        debug_dump_address_list(
                context,
                "src ",
                base,
                offset,
                bytecount,
                selector->source_address_count);

    offset =
        debug_dump_address_list(
                context,
                "dst ",
                base,
                offset,
                bytecount,
                selector->destination_address_count);

    offset =
        debug_dump_port_list(
                context,
                "src ",
                base,
                offset,
                bytecount,
                selector->source_port_count);

    (void)
        debug_dump_port_list(
                context,
                "dst ",
                base,
                offset,
                bytecount,
                selector->destination_port_count);
}


void
debug_dump_ip_selector_group(
        void *context,
        const void *data,
        unsigned bytecount)
{
    const struct IPSelectorGroup *selector_group = data;

    if (bytecount < sizeof *selector_group)
    {
        DEBUG_DUMP_LINE(
                context,
                "Selector_Group %p: Given bytecount %u too small.",
                selector_group,
                bytecount);
    }
    else
    if (bytecount < selector_group->bytecount)
    {
        DEBUG_DUMP_LINE(
                context,
                "Selector_Group %p: Given bytecount %u too small.",
                selector_group,
                bytecount);
    }
    else
    {
        const uint8_t *base = (void *) selector_group;
        const uint32_t selector_count = selector_group->selector_count;
        const unsigned group_bytecount = selector_group->bytecount;
        unsigned offset = sizeof *selector_group;
        int i;

        DEBUG_DUMP_LINE(
                context,
                "Selector_Group %p: selector count %u, bytecount %u:",
                selector_group,
                selector_count,
                group_bytecount);

        for (i = 0; i < selector_count && offset < group_bytecount; i++)
        {
            const struct IPSelector *selector = (void *)(base + offset);

            offset += selector->bytecount;

            if (offset <= group_bytecount)
            {
                debug_dump_ip_selector(context, selector, selector->bytecount);
            }
            else
            {
                DEBUG_DUMP_LINE(
                        context,
                        "offset %u exceeded bytecount %u",
                        offset,
                        group_bytecount);
            }
        }
    }
}


void
ip_selector_match_debug_log_fields(
        const struct IPSelectorFields *fields)
{
    DEBUG_LOW(
            "Selector fields %p: "
            "ip version %d ip protocol %d from %pI6c:%d to %pI6c :%d.",
            fields,
            fields->ip_version,
            fields->ip_protocol,
            fields->source_address,
            fields->source_port,
            fields->destination_address,
            fields->destination_port);
}


static int
ip_selector_match_port_value(
        const struct IPSelectorPort *port_selector,
        int port)
{
    if (port == IP_SELECTOR_PORT_NONE)
    {
        return 1;
    }

    if (port_selector->begin == IP_SELECTOR_PORT_MIN &&
        port_selector->end == IP_SELECTOR_PORT_MAX)
    {
        return 1;
    }

    if (port == IP_SELECTOR_PORT_OPAQUE)
    {
        if (port_selector->begin == IP_SELECTOR_PORT_MAX &&
            port_selector->end == IP_SELECTOR_PORT_MIN)
        {
            return 1;
        }
    }

    if (port >= port_selector->begin && port <= port_selector->end)
    {
        return 1;
    }

    return 0;
}


static int
ip_selector_match_endpoint_match(
        const uint8_t *base,
        unsigned *offset,
        int endpoint_count,
        int ip_version,
        int ip_protocol,
        const uint8_t address[16],
        int port)
{
    const struct IPSelectorEndpoint *endpoints =
        (const struct IPSelectorEndpoint *) (base + *offset);

    int index;

    *offset += endpoint_count * sizeof (struct IPSelectorEndpoint);

    if (endpoint_count == 0)
    {
        return 1;
    }

    for (index = 0; index < endpoint_count; index++)
    {
        const struct IPSelectorEndpoint *endpoint_selector = endpoints + index;

        if (endpoint_selector->ip_protocol != 0 &&
            endpoint_selector->ip_protocol != ip_protocol)
        {
            continue;
        }

        if (endpoint_selector->ip_version != 0 &&
            endpoint_selector->ip_version != ip_version)
        {
            continue;
        }

        if (memcmp(address, endpoint_selector->address.begin, 16) < 0 ||
            memcmp(address, endpoint_selector->address.end, 16) > 0)
        {
            continue;
        }

        if (ip_selector_match_port_value(&endpoint_selector->port, port) == 0)
        {
            continue;
        }

        return 1;
    }

    return 0;
}


static int
ip_selector_match_address_match(
        const uint8_t *base,
        unsigned *offset,
        int address_count,
        const uint8_t address[16])
{
    const struct IPSelectorAddress *addresses =
        (const struct IPSelectorAddress *) (base + *offset);
    int index;

    *offset += address_count * sizeof (struct IPSelectorAddress);

    if (address_count == 0)
    {
        return 1;
    }

    for (index = 0; index < address_count; index++)
    {
        const struct IPSelectorAddress *address_selector = addresses + index;

        if (memcmp(address, address_selector->begin, 16) >= 0 &&
            memcmp(address, address_selector->end, 16) <= 0)
        {
            return 1;
        }
    }

    return 0;
}


static int
ip_selector_match_port_match(
        const uint8_t *base,
        unsigned *offset,
        int port_count,
        int32_t port)
{
    const struct IPSelectorPort *ports =
            (const struct IPSelectorPort *) (base + *offset);

    int index;

    *offset += port_count * sizeof (struct IPSelectorPort);

    if (port_count == 0)
    {
        return 1;
    }

    for (index = 0; index < port_count; index++)
    {
        if (ip_selector_match_port_value(&ports[index], port) == 1)
        {
            return 1;
        }
    }


    return 0;
}


static int
ip_selector_match_interface_name_match(
        const uint8_t *base,
        unsigned *offset,
        int interface_bytecount,
        const char *interface_name)
{
    const char *interface = (const char *) (base + *offset);

    *offset += interface_bytecount;

    if (interface_bytecount == 0)
    {
        return 1;
    }

    if (interface_name == NULL)
    {
        return 0;
    }

    if (strcmp(interface, interface_name) == 0)
    {
        return 1;
    }

    return 0;
}


static int
ip_selector_match_selector_match(
        const struct IPSelector *selector,
        const struct IPSelectorFields *fields,
        const char *interface_name)
{
    const uint8_t *base = (void *) selector;
    unsigned offset = sizeof *selector;
    int match = 1;

    DEBUG_LOW(
            "Fields %p: matching selector %p.",
            fields,
            selector);

    ip_selector_match_debug_log_selector(selector);

    {
        if (selector->ip_version != 0 &&
            selector->ip_version != fields->ip_version)
        {
            DEBUG_LOW(
                    "Fields %p: No match for protocol.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (selector->ip_protocol != 0 &&
            selector->ip_protocol != fields->ip_protocol)
        {
            DEBUG_LOW(
                    "Fields %p: No match for protocol.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_endpoint_match(
                    base,
                    &offset,
                    selector->source_endpoint_count,
                    fields->ip_version,
                    fields->ip_protocol,
                    fields->source_address,
                    fields->source_port))
        {
            DEBUG_LOW(
                    "Fields %p: No match for source endpoint.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_endpoint_match(
                    base,
                    &offset,
                    selector->destination_endpoint_count,
                    fields->ip_version,
                    fields->ip_protocol,
                    fields->destination_address,
                    fields->destination_port))
        {
            DEBUG_LOW(
                    "Fields %p: No match for destination endpoint.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_address_match(
                    base,
                    &offset,
                    selector->source_address_count,
                    fields->source_address))
        {
            DEBUG_LOW(
                    "Fields %p: No match for source address.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_address_match(
                    base,
                    &offset,
                    selector->destination_address_count,
                    fields->destination_address))
        {
            DEBUG_LOW(
                    "Fields %p: No match for destination address.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_port_match(
                    base,
                    &offset,
                    selector->source_port_count,
                    fields->source_port))
        {
            DEBUG_LOW(
                    "Fields %p: No match for source port.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_port_match(
                    base,
                    &offset,
                    selector->destination_port_count,
                    fields->destination_port))
        {
            DEBUG_LOW(
                    "Fields %p: No match for destination port.",
                    fields);

            match = 0;
        }
    }

    if (match)
    {
        if (!ip_selector_match_interface_name_match(
                    base,
                    &offset,
                    selector->interface_bytecount,
                    interface_name))
        {
            DEBUG_LOW(
                    "Fields %p: No match for interface name '%s'.",
                    fields,
                    interface_name);

            return 0;
        }
    }

    return match;
}


int
ip_selector_match_fields_to_group(
        const struct IPSelectorGroup *selector_group,
        const struct IPSelectorFields *fields,
        const char *interface_name)
{
    const char *first = (void *) selector_group;
    unsigned offset = sizeof *selector_group;
    int i;
    bool match = true;

    DEBUG_LOW("Fields %p: Match Group %p:", fields, selector_group);
    ip_selector_match_debug_log_fields(fields);

    for (i = 0; i < selector_group->selector_count; i++)
    {
        struct IPSelector *selector = (void *) (first + offset);

        if (ip_selector_match_selector_match(selector, fields, interface_name))
        {
            match = true;
            DEBUG_LOW(
                    "Fields %p on selector %p: match.",
                    fields,
                    selector);
            break;
        }
        else
        {
            DEBUG_LOW(
                    "Fields %p on selector %p: no match.",
                    fields,
                    selector);
        }

        offset += selector->bytecount;
        match = false;
    }

    if (match == true)
    {
        DEBUG_LOW(
                "Fields %p matched Selector Group %p.",
                fields,
                selector_group);
    }
    else
    {
        DEBUG_LOW(
                "Fields %p did not match Selector Group %p.",
                fields,
                selector_group);
    }

    return (int) match;
}

