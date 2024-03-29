/*
 * base.c - basic devicetree functions
 *
 * Copyright (c) 2012 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * based on Linux devicetree support
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <common.h>
#include <of.h>
#include <errno.h>
#include <malloc.h>
#include <init.h>
#include <memory.h>
#include <sizes.h>
#include <linux/ctype.h>
#include <linux/amba/bus.h>
#include <linux/err.h>

/*
 * Iterate over all nodes of a tree. As a devicetree does not
 * have a dedicated list head, the start node (usually the root
 * node) will not be iterated over.
 */
static inline struct device_node *of_next_node(struct device_node *node)
{
	struct device_node *next;

	next = list_first_entry(&node->list, struct device_node, list);

	return next->parent ? next : NULL;
}

#define of_tree_for_each_node_from(node, from) \
	for (node = of_next_node(from); node; node = of_next_node(node))

/**
 * struct alias_prop - Alias property in 'aliases' node
 * @link:	List node to link the structure in aliases_lookup list
 * @alias:	Alias property name
 * @np:		Pointer to device_node that the alias stands for
 * @id:		Index value from end of alias name
 * @stem:	Alias string without the index
 *
 * The structure represents one alias property of 'aliases' node as
 * an entry in aliases_lookup list.
 */
struct alias_prop {
	struct list_head link;
	const char *alias;
	struct device_node *np;
	int id;
	char stem[0];
};

static LIST_HEAD(aliases_lookup);

static LIST_HEAD(phandle_list);

struct device_node *root_node;

struct device_node *of_aliases;

int of_n_addr_cells(struct device_node *np)
{
	const __be32 *ip;

	do {
		if (np->parent)
			np = np->parent;
		ip = of_get_property(np, "#address-cells", NULL);
		if (ip)
			return be32_to_cpup(ip);
	} while (np->parent);
	/* No #address-cells property for the root node */
	return OF_ROOT_NODE_ADDR_CELLS_DEFAULT;
}
EXPORT_SYMBOL(of_n_addr_cells);

int of_n_size_cells(struct device_node *np)
{
	const __be32 *ip;

	do {
		if (np->parent)
			np = np->parent;
		ip = of_get_property(np, "#size-cells", NULL);
		if (ip)
			return be32_to_cpup(ip);
	} while (np->parent);
	/* No #size-cells property for the root node */
	return OF_ROOT_NODE_SIZE_CELLS_DEFAULT;
}
EXPORT_SYMBOL(of_n_size_cells);

static void of_bus_default_count_cells(struct device_node *dev,
				       int *addrc, int *sizec)
{
	if (addrc)
		*addrc = of_n_addr_cells(dev);
	if (sizec)
		*sizec = of_n_size_cells(dev);
}

static void of_bus_count_cells(struct device_node *dev,
			int *addrc, int *sizec)
{
	of_bus_default_count_cells(dev, addrc, sizec);
}

struct property *of_find_property(const struct device_node *np,
				  const char *name, int *lenp)
{
	struct property *pp;

	if (!np)
		return NULL;

	list_for_each_entry(pp, &np->properties, list)
		if (of_prop_cmp(pp->name, name) == 0) {
			if (lenp)
				*lenp = pp->length;
			return pp;
		}

	return NULL;
}
EXPORT_SYMBOL(of_find_property);

static void of_alias_add(struct alias_prop *ap, struct device_node *np,
			 int id, const char *stem, int stem_len)
{
	ap->np = np;
	ap->id = id;
	strncpy(ap->stem, stem, stem_len);
	ap->stem[stem_len] = 0;
	list_add_tail(&ap->link, &aliases_lookup);
	pr_debug("adding DT alias:%s: stem=%s id=%i node=%s\n",
		 ap->alias, ap->stem, ap->id, np->full_name);
}

/**
 * of_alias_scan - Scan all properties of 'aliases' node
 *
 * The function scans all the properties of 'aliases' node and populates
 * the global lookup table with the properties.  It returns the
 * number of alias_prop found, or error code in error case.
 */
void of_alias_scan(void)
{
	struct property *pp;
	struct alias_prop *app, *tmp;

	list_for_each_entry_safe(app, tmp, &aliases_lookup, link)
		free(app);

	INIT_LIST_HEAD(&aliases_lookup);

	if (!root_node)
		return;

	of_aliases = of_find_node_by_path("/aliases");
	if (!of_aliases)
		return;

	list_for_each_entry(pp, &of_aliases->properties, list) {
		const char *start = pp->name;
		const char *end = start + strlen(start);
		struct device_node *np;
		struct alias_prop *ap;
		int id, len;

		/* Skip those we do not want to proceed */
		if (!of_prop_cmp(pp->name, "name") ||
		    !of_prop_cmp(pp->name, "phandle") ||
		    !of_prop_cmp(pp->name, "linux,phandle"))
			continue;

		np = of_find_node_by_path(pp->value);
		if (!np)
			continue;

		/* walk the alias backwards to extract the id and work out
		 * the 'stem' string */
		while (isdigit(*(end-1)) && end > start)
			end--;
		len = end - start;

		id = simple_strtol(end, 0, 10);
		if (id < 0)
			continue;

		/* Allocate an alias_prop with enough space for the stem */
		ap = xzalloc(sizeof(*ap) + len + 1);
		if (!ap)
			continue;
		ap->alias = start;
		of_alias_add(ap, np, id, start, len);
	}
}
EXPORT_SYMBOL(of_alias_scan);

/**
 * of_alias_get_id - Get alias id for the given device_node
 * @np:		Pointer to the given device_node
 * @stem:	Alias stem of the given device_node
 *
 * The function travels the lookup table to get alias id for the given
 * device_node and alias stem.  It returns the alias id if find it.
 */
int of_alias_get_id(struct device_node *np, const char *stem)
{
	struct alias_prop *app;
	int id = -ENODEV;

	list_for_each_entry(app, &aliases_lookup, link) {
		if (of_node_cmp(app->stem, stem) != 0)
			continue;

		if (np == app->np) {
			id = app->id;
			break;
		}
	}

	return id;
}
EXPORT_SYMBOL_GPL(of_alias_get_id);

const char *of_alias_get(struct device_node *np)
{
	struct property *pp;

	list_for_each_entry(pp, &of_aliases->properties, list) {
		if (!of_node_cmp(np->full_name, pp->value))
			return pp->name;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(of_alias_get);

u64 of_translate_address(struct device_node *node, const __be32 *in_addr)
{
	struct property *p;
	u64 addr = be32_to_cpu(*in_addr);

	while (1) {
		int na, nc;

		if (!node->parent)
			return addr;

		node = node->parent;
		p = of_find_property(node, "ranges", NULL);
		if (!p && node->parent)
			return OF_BAD_ADDR;
		of_bus_count_cells(node, &na, &nc);
		if (na != 1 || nc != 1) {
			printk("%s: #size-cells != 1 or #address-cells != 1 "
					"currently not supported\n", node->name);
			return OF_BAD_ADDR;
		}
	}
}
EXPORT_SYMBOL(of_translate_address);

/*
 * of_find_node_by_phandle - Find a node given a phandle
 * @handle:    phandle of the node to find
 */
struct device_node *of_find_node_by_phandle(phandle phandle)
{
	struct device_node *node;

	list_for_each_entry(node, &phandle_list, phandles)
		if (node->phandle == phandle)
			return node;
	return NULL;
}
EXPORT_SYMBOL(of_find_node_by_phandle);

/*
 * Find a property with a given name for a given node
 * and return the value.
 */
const void *of_get_property(const struct device_node *np, const char *name,
			 int *lenp)
{
	struct property *pp = of_find_property(np, name, lenp);

	return pp ? pp->value : NULL;
}
EXPORT_SYMBOL(of_get_property);

/** Checks if the given "compat" string matches one of the strings in
 * the device's "compatible" property
 */
int of_device_is_compatible(const struct device_node *device,
		const char *compat)
{
	const char *cp;
	int cplen, l;

	cp = of_get_property(device, "compatible", &cplen);
	if (cp == NULL)
		return 0;
	while (cplen > 0) {
		if (of_compat_cmp(cp, compat, strlen(compat)) == 0)
			return 1;
		l = strlen(cp) + 1;
		cp += l;
		cplen -= l;
	}

	return 0;
}
EXPORT_SYMBOL(of_device_is_compatible);

/**
 *	of_find_node_by_name - Find a node by its "name" property
 *	@from:	The node to start searching from or NULL, the node
 *		you pass will not be searched, only the next one
 *		will; typically, you pass what the previous call
 *		returned.
 *	@name:	The name string to match against
 *
 *	Returns a pointer to the node found or NULL.
 */
struct device_node *of_find_node_by_name(struct device_node *from,
	const char *name)
{
	struct device_node *np;

	if (!from)
		from = root_node;

	of_tree_for_each_node_from(np, from)
		if (np->name && !of_node_cmp(np->name, name))
			return np;

	return NULL;
}
EXPORT_SYMBOL(of_find_node_by_name);

/**
 *	of_find_compatible_node - Find a node based on type and one of the
 *                                tokens in its "compatible" property
 *	@from:		The node to start searching from or NULL, the node
 *			you pass will not be searched, only the next one
 *			will; typically, you pass what the previous call
 *			returned.
 *	@type:		The type string to match "device_type" or NULL to ignore
 *                      (currently always ignored in barebox)
 *	@compatible:	The string to match to one of the tokens in the device
 *			"compatible" list.
 *
 *	Returns a pointer to the node found or NULL.
 */
struct device_node *of_find_compatible_node(struct device_node *from,
	const char *type, const char *compatible)
{
	struct device_node *np;

	if (!from)
		from = root_node;

	of_tree_for_each_node_from(np, from)
		if (of_device_is_compatible(np, compatible))
			return np;

	return NULL;
}
EXPORT_SYMBOL(of_find_compatible_node);

/**
 *	of_find_node_with_property - Find a node which has a property with
 *                                   the given name.
 *	@from:		The node to start searching from or NULL, the node
 *			you pass will not be searched, only the next one
 *			will; typically, you pass what the previous call
 *			returned.
 *	@prop_name:	The name of the property to look for.
 *
 *	Returns a pointer to the node found or NULL.
 */
struct device_node *of_find_node_with_property(struct device_node *from,
	const char *prop_name)
{
	struct device_node *np;

	if (!from)
		from = root_node;

	of_tree_for_each_node_from(np, from) {
		struct property *pp = of_find_property(np, prop_name, NULL);
		if (pp)
			return np;
	}

	return NULL;
}
EXPORT_SYMBOL(of_find_node_with_property);

/**
 * of_match_node - Tell if an device_node has a matching of_match structure
 *      @matches:       array of of device match structures to search in
 *      @node:          the of device structure to match against
 *
 *      Low level utility function used by device matching.
 */
const struct of_device_id *of_match_node(const struct of_device_id *matches,
					 const struct device_node *node)
{
	if (!matches || !node)
		return NULL;

	while (matches->compatible) {
		if (of_device_is_compatible(node, matches->compatible) == 1)
			return matches;
		matches++;
	}

	return NULL;
}

/**
 *	of_find_matching_node_and_match - Find a node based on an of_device_id
 *					  match table.
 *	@from:		The node to start searching from or NULL, the node
 *			you pass will not be searched, only the next one
 *			will; typically, you pass what the previous call
 *			returned.
 *	@matches:	array of of device match structures to search in
 *	@match		Updated to point at the matches entry which matched
 *
 *	Returns a pointer to the node found or NULL.
 */
struct device_node *of_find_matching_node_and_match(struct device_node *from,
					const struct of_device_id *matches,
					const struct of_device_id **match)
{
	struct device_node *np;

	if (match)
		*match = NULL;

	if (!from)
		from = root_node;

	of_tree_for_each_node_from(np, from) {
		const struct of_device_id *m = of_match_node(matches, np);
		if (m) {
			if (match)
				*match = m;
			return np;
		}
	}

	return NULL;
}
EXPORT_SYMBOL(of_find_matching_node_and_match);

int of_match(struct device_d *dev, struct driver_d *drv)
{
	const struct of_device_id *id;

	id = of_match_node(drv->of_compatible, dev->device_node);
	if (!id)
		return 1;

	dev->of_id_entry = id;

	return 0;
}
EXPORT_SYMBOL(of_match);

/**
 * of_find_property_value_of_size
 *
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @len:	requested length of property value
 *
 * Search for a property in a device node and valid the requested size.
 * Returns the property value on success, -EINVAL if the property does not
 *  exist, -ENODATA if property does not have a value, and -EOVERFLOW if the
 * property data isn't large enough.
 *
 */
static void *of_find_property_value_of_size(const struct device_node *np,
			const char *propname, u32 len)
{
	struct property *prop = of_find_property(np, propname, NULL);

	if (!prop)
		return ERR_PTR(-EINVAL);
	if (!prop->value)
		return ERR_PTR(-ENODATA);
	if (len > prop->length)
		return ERR_PTR(-EOVERFLOW);

	return prop->value;
}

/**
 * of_property_read_u32_index - Find and read a u32 from a multi-value property.
 *
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @index:	index of the u32 in the list of values
 * @out_value:	pointer to return value, modified only if no error.
 *
 * Search for a property in a device node and read nth 32-bit value from
 * it. Returns 0 on success, -EINVAL if the property does not exist,
 * -ENODATA if property does not have a value, and -EOVERFLOW if the
 * property data isn't large enough.
 *
 * The out_value is modified only if a valid u32 value can be decoded.
 */
int of_property_read_u32_index(const struct device_node *np,
				       const char *propname,
				       u32 index, u32 *out_value)
{
	const u32 *val = of_find_property_value_of_size(np, propname,
					((index + 1) * sizeof(*out_value)));

	if (IS_ERR(val))
		return PTR_ERR(val);

	*out_value = be32_to_cpup(((__be32 *)val) + index);
	return 0;
}
EXPORT_SYMBOL_GPL(of_property_read_u32_index);

/**
 * of_property_read_u8_array - Find and read an array of u8 from a property.
 *
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @out_value:	pointer to return value, modified only if return value is 0.
 * @sz:		number of array elements to read
 *
 * Search for a property in a device node and read 8-bit value(s) from
 * it. Returns 0 on success, -EINVAL if the property does not exist,
 * -ENODATA if property does not have a value, and -EOVERFLOW if the
 * property data isn't large enough.
 *
 * dts entry of array should be like:
 *	property = /bits/ 8 <0x50 0x60 0x70>;
 *
 * The out_value is modified only if a valid u8 value can be decoded.
 */
int of_property_read_u8_array(const struct device_node *np,
			const char *propname, u8 *out_values, size_t sz)
{
	const u8 *val = of_find_property_value_of_size(np, propname,
						(sz * sizeof(*out_values)));

	if (IS_ERR(val))
		return PTR_ERR(val);

	while (sz--)
		*out_values++ = *val++;
	return 0;
}
EXPORT_SYMBOL_GPL(of_property_read_u8_array);

/**
 * of_property_read_u16_array - Find and read an array of u16 from a property.
 *
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @out_value:	pointer to return value, modified only if return value is 0.
 * @sz:		number of array elements to read
 *
 * Search for a property in a device node and read 16-bit value(s) from
 * it. Returns 0 on success, -EINVAL if the property does not exist,
 * -ENODATA if property does not have a value, and -EOVERFLOW if the
 * property data isn't large enough.
 *
 * dts entry of array should be like:
 *	property = /bits/ 16 <0x5000 0x6000 0x7000>;
 *
 * The out_value is modified only if a valid u16 value can be decoded.
 */
int of_property_read_u16_array(const struct device_node *np,
			const char *propname, u16 *out_values, size_t sz)
{
	const __be16 *val = of_find_property_value_of_size(np, propname,
						(sz * sizeof(*out_values)));

	if (IS_ERR(val))
		return PTR_ERR(val);

	while (sz--)
		*out_values++ = be16_to_cpup(val++);
	return 0;
}
EXPORT_SYMBOL_GPL(of_property_read_u16_array);

/**
 * of_property_read_u32_array - Find and read an array of 32 bit integers
 * from a property.
 *
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @out_value:	pointer to return value, modified only if return value is 0.
 * @sz:		number of array elements to read
 *
 * Search for a property in a device node and read 32-bit value(s) from
 * it. Returns 0 on success, -EINVAL if the property does not exist,
 * -ENODATA if property does not have a value, and -EOVERFLOW if the
 * property data isn't large enough.
 *
 * The out_value is modified only if a valid u32 value can be decoded.
 */
int of_property_read_u32_array(const struct device_node *np,
			       const char *propname, u32 *out_values,
			       size_t sz)
{
	const __be32 *val = of_find_property_value_of_size(np, propname,
						(sz * sizeof(*out_values)));

	if (IS_ERR(val))
		return PTR_ERR(val);

	while (sz--)
		*out_values++ = be32_to_cpup(val++);
	return 0;
}
EXPORT_SYMBOL_GPL(of_property_read_u32_array);

/**
 * of_property_read_u64 - Find and read a 64 bit integer from a property
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @out_value:	pointer to return value, modified only if return value is 0.
 *
 * Search for a property in a device node and read a 64-bit value from
 * it. Returns 0 on success, -EINVAL if the property does not exist,
 * -ENODATA if property does not have a value, and -EOVERFLOW if the
 * property data isn't large enough.
 *
 * The out_value is modified only if a valid u64 value can be decoded.
 */
int of_property_read_u64(const struct device_node *np, const char *propname,
			 u64 *out_value)
{
	const __be32 *val = of_find_property_value_of_size(np, propname,
						sizeof(*out_value));

	if (IS_ERR(val))
		return PTR_ERR(val);

	*out_value = of_read_number(val, 2);
	return 0;
}
EXPORT_SYMBOL_GPL(of_property_read_u64);

/**
 * of_property_read_string - Find and read a string from a property
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @out_string:	pointer to null terminated return string, modified only if
 *		return value is 0.
 *
 * Search for a property in a device tree node and retrieve a null
 * terminated string value (pointer to data, not a copy). Returns 0 on
 * success, -EINVAL if the property does not exist, -ENODATA if property
 * does not have a value, and -EILSEQ if the string is not null-terminated
 * within the length of the property data.
 *
 * The out_string pointer is modified only if a valid string can be decoded.
 */
int of_property_read_string(struct device_node *np, const char *propname,
				const char **out_string)
{
	struct property *prop = of_find_property(np, propname, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	if (strnlen(prop->value, prop->length) >= prop->length)
		return -EILSEQ;
	*out_string = prop->value;
	return 0;
}
EXPORT_SYMBOL_GPL(of_property_read_string);

/**
 * of_property_read_string_index - Find and read a string from a multiple
 * strings property.
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 * @index:	index of the string in the list of strings
 * @out_string:	pointer to null terminated return string, modified only if
 *		return value is 0.
 *
 * Search for a property in a device tree node and retrieve a null
 * terminated string value (pointer to data, not a copy) in the list of strings
 * contained in that property.
 * Returns 0 on success, -EINVAL if the property does not exist, -ENODATA if
 * property does not have a value, and -EILSEQ if the string is not
 * null-terminated within the length of the property data.
 *
 * The out_string pointer is modified only if a valid string can be decoded.
 */
int of_property_read_string_index(struct device_node *np, const char *propname,
				  int index, const char **output)
{
	struct property *prop = of_find_property(np, propname, NULL);
	int i = 0;
	size_t l = 0, total = 0;
	const char *p;

	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	if (strnlen(prop->value, prop->length) >= prop->length)
		return -EILSEQ;

	p = prop->value;

	for (i = 0; total < prop->length; total += l, p += l) {
		l = strlen(p) + 1;
		if (i++ == index) {
			*output = p;
			return 0;
		}
	}
	return -ENODATA;
}
EXPORT_SYMBOL_GPL(of_property_read_string_index);

/**
 * of_property_match_string() - Find string in a list and return index
 * @np: pointer to node containing string list property
 * @propname: string list property name
 * @string: pointer to string to search for in string list
 *
 * This function searches a string list property and returns the index
 * of a specific string value.
 */
int of_property_match_string(struct device_node *np, const char *propname,
			     const char *string)
{
	struct property *prop = of_find_property(np, propname, NULL);
	size_t l;
	int i;
	const char *p, *end;

	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	p = prop->value;
	end = p + prop->length;

	for (i = 0; p < end; i++, p += l) {
		l = strlen(p) + 1;
		if (p + l > end)
			return -EILSEQ;
		pr_debug("comparing %s with %s\n", string, p);
		if (strcmp(string, p) == 0)
			return i; /* Found it; return index */
	}
	return -ENODATA;
}
EXPORT_SYMBOL_GPL(of_property_match_string);

/**
 * of_property_count_strings - Find and return the number of strings from a
 * multiple strings property.
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 *
 * Search for a property in a device tree node and retrieve the number of null
 * terminated string contain in it. Returns the number of strings on
 * success, -EINVAL if the property does not exist, -ENODATA if property
 * does not have a value, and -EILSEQ if the string is not null-terminated
 * within the length of the property data.
 */
int of_property_count_strings(struct device_node *np, const char *propname)
{
	struct property *prop = of_find_property(np, propname, NULL);
	int i = 0;
	size_t l = 0, total = 0;
	const char *p;

	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	if (strnlen(prop->value, prop->length) >= prop->length)
		return -EILSEQ;

	p = prop->value;

	for (i = 0; total < prop->length; total += l, p += l, i++)
		l = strlen(p) + 1;

	return i;
}
EXPORT_SYMBOL_GPL(of_property_count_strings);

const __be32 *of_prop_next_u32(struct property *prop, const __be32 *cur,
			u32 *pu)
{
	const void *curv = cur;

	if (!prop)
		return NULL;

	if (!cur) {
		curv = prop->value;
		goto out_val;
	}

	curv += sizeof(*cur);
	if (curv >= prop->value + prop->length)
		return NULL;

out_val:
	*pu = be32_to_cpup(curv);
	return curv;
}
EXPORT_SYMBOL_GPL(of_prop_next_u32);

const char *of_prop_next_string(struct property *prop, const char *cur)
{
	const void *curv = cur;

	if (!prop)
		return NULL;

	if (!cur)
		return prop->value;

	curv += strlen(cur) + 1;
	if (curv >= prop->value + prop->length)
		return NULL;

	return curv;
}
EXPORT_SYMBOL_GPL(of_prop_next_string);

/**
 * of_property_write_bool - Create/Delete empty (bool) property.
 *
 * @np:		device node from which the property is to be set.
 * @propname:	name of the property to be set.
 *
 * Search for a property in a device node and create or delete the property.
 * If the property already exists and write value is false, the property is
 * deleted. If write value is true and the property does not exist, it is
 * created. Returns 0 on success, -ENOMEM if the property or array
 * of elements cannot be created.
 */
int of_property_write_bool(struct device_node *np, const char *propname,
			   const bool value)
{
	struct property *prop = of_find_property(np, propname, NULL);

	if (!value) {
		if (prop)
			of_delete_property(prop);
		return 0;
	}

	if (!prop)
		prop = of_new_property(np, propname, NULL, 0);
	if (!prop)
		return -ENOMEM;

	return 0;
}

/**
 * of_property_write_u8_array - Write an array of u8 to a property. If
 * the property does not exist, it will be created and appended to the given
 * device node.
 *
 * @np:		device node to which the property value is to be written.
 * @propname:	name of the property to be written.
 * @values:	pointer to array elements to write.
 * @sz:		number of array elements to write.
 *
 * Search for a property in a device node and write 8-bit value(s) to
 * it. If the property does not exist, it will be created and appended to
 * the device node. Returns 0 on success, -ENOMEM if the property or array
 * of elements cannot be created.
 */
int of_property_write_u8_array(struct device_node *np,
			       const char *propname, const u8 *values,
			       size_t sz)
{
	struct property *prop = of_find_property(np, propname, NULL);
	u8 *val;

	if (prop)
		of_delete_property(prop);

	prop = of_new_property(np, propname, NULL, sizeof(*val) * sz);
	if (!prop)
		return -ENOMEM;

	val = prop->value;
	while (sz--)
		*val++ = *values++;

	return 0;
}

/**
 * of_property_write_u16_array - Write an array of u16 to a property. If
 * the property does not exist, it will be created and appended to the given
 * device node.
 *
 * @np:		device node to which the property value is to be written.
 * @propname:	name of the property to be written.
 * @values:	pointer to array elements to write.
 * @sz:		number of array elements to write.
 *
 * Search for a property in a device node and write 16-bit value(s) to
 * it. If the property does not exist, it will be created and appended to
 * the device node. Returns 0 on success, -ENOMEM if the property or array
 * of elements cannot be created.
 */
int of_property_write_u16_array(struct device_node *np,
				const char *propname, const u16 *values,
				size_t sz)
{
	struct property *prop = of_find_property(np, propname, NULL);
	__be16 *val;

	if (prop)
		of_delete_property(prop);

	prop = of_new_property(np, propname, NULL, sizeof(*val) * sz);
	if (!prop)
		return -ENOMEM;

	val = prop->value;
	while (sz--)
		*val++ = cpu_to_be16(*values++);

	return 0;
}

/**
 * of_property_write_u32_array - Write an array of u32 to a property. If
 * the property does not exist, it will be created and appended to the given
 * device node.
 *
 * @np:		device node to which the property value is to be written.
 * @propname:	name of the property to be written.
 * @values:	pointer to array elements to write.
 * @sz:		number of array elements to write.
 *
 * Search for a property in a device node and write 32-bit value(s) to
 * it. If the property does not exist, it will be created and appended to
 * the device node. Returns 0 on success, -ENOMEM if the property or array
 * of elements cannot be created.
 */
int of_property_write_u32_array(struct device_node *np,
				const char *propname, const u32 *values,
				size_t sz)
{
	struct property *prop = of_find_property(np, propname, NULL);
	__be32 *val;

	if (prop)
		of_delete_property(prop);

	prop = of_new_property(np, propname, NULL, sizeof(*val) * sz);
	if (!prop)
		return -ENOMEM;

	val = prop->value;
	while (sz--)
		*val++ = cpu_to_be32(*values++);

	return 0;
}

/**
 * of_property_write_u64_array - Write an array of u64 to a property. If
 * the property does not exist, it will be created and appended to the given
 * device node.
 *
 * @np:		device node to which the property value is to be written.
 * @propname:	name of the property to be written.
 * @values:	pointer to array elements to write.
 * @sz:		number of array elements to write.
 *
 * Search for a property in a device node and write 64-bit value(s) to
 * it. If the property does not exist, it will be created and appended to
 * the device node. Returns 0 on success, -ENOMEM if the property or array
 * of elements cannot be created.
 */
int of_property_write_u64_array(struct device_node *np,
				const char *propname, const u64 *values,
				size_t sz)
{
	struct property *prop = of_find_property(np, propname, NULL);
	__be32 *val;

	if (prop)
		of_delete_property(prop);

	prop = of_new_property(np, propname, NULL, 2 * sizeof(*val) * sz);
	if (!prop)
		return -ENOMEM;

	val = prop->value;
	while (sz--) {
		of_write_number(val, *values++, 2);
		val += 2;
	}

	return 0;
}

/**
 * of_parse_phandle - Resolve a phandle property to a device_node pointer
 * @np: Pointer to device node holding phandle property
 * @phandle_name: Name of property holding a phandle value
 * @index: For properties holding a table of phandles, this is the index into
 *         the table
 *
 * Returns the device_node pointer found or NULL.
 */
struct device_node *of_parse_phandle(const struct device_node *np,
				     const char *phandle_name, int index)
{
	const __be32 *phandle;
	int size;

	phandle = of_get_property(np, phandle_name, &size);
	if ((!phandle) || (size < sizeof(*phandle) * (index + 1)))
		return NULL;

	return of_find_node_by_phandle(be32_to_cpup(phandle + index));
}
EXPORT_SYMBOL(of_parse_phandle);

/**
 * of_parse_phandle_with_args() - Find a node pointed by phandle in a list
 * @np:		pointer to a device tree node containing a list
 * @list_name:	property name that contains a list
 * @cells_name:	property name that specifies phandles' arguments count
 * @index:	index of a phandle to parse out
 * @out_args:	optional pointer to output arguments structure (will be filled)
 *
 * This function is useful to parse lists of phandles and their arguments.
 * Returns 0 on success and fills out_args, on error returns appropriate
 * errno value.
 *
 * Example:
 *
 * phandle1: node1 {
 * 	#list-cells = <2>;
 * }
 *
 * phandle2: node2 {
 * 	#list-cells = <1>;
 * }
 *
 * node3 {
 * 	list = <&phandle1 1 2 &phandle2 3>;
 * }
 *
 * To get a device_node of the `node2' node you may call this:
 * of_parse_phandle_with_args(node3, "list", "#list-cells", 1, &args);
 */
static int __of_parse_phandle_with_args(const struct device_node *np,
					const char *list_name,
					const char *cells_name, int index,
					struct of_phandle_args *out_args)
{
	const __be32 *list, *list_end;
	int rc = 0, size, cur_index = 0;
	uint32_t count = 0;
	struct device_node *node = NULL;
	phandle phandle;

	/* Retrieve the phandle list property */
	list = of_get_property(np, list_name, &size);
	if (!list)
		return -ENOENT;
	list_end = list + size / sizeof(*list);

	/* Loop over the phandles until all the requested entry is found */
	while (list < list_end) {
		rc = -EINVAL;
		count = 0;

		/*
		 * If phandle is 0, then it is an empty entry with no
		 * arguments.  Skip forward to the next entry.
		 */
		phandle = be32_to_cpup(list++);
		if (phandle) {
			/*
			 * Find the provider node and parse the #*-cells
			 * property to determine the argument length
			 */
			node = of_find_node_by_phandle(phandle);
			if (!node) {
				pr_err("%s: could not find phandle\n",
					 np->full_name);
				goto err;
			}
			if (of_property_read_u32(node, cells_name, &count)) {
				pr_err("%s: could not get %s for %s\n",
					 np->full_name, cells_name,
					 node->full_name);
				goto err;
			}

			/*
			 * Make sure that the arguments actually fit in the
			 * remaining property data length
			 */
			if (list + count > list_end) {
				pr_err("%s: arguments longer than property\n",
					 np->full_name);
				goto err;
			}
		}

		/*
		 * All of the error cases above bail out of the loop, so at
		 * this point, the parsing is successful. If the requested
		 * index matches, then fill the out_args structure and return,
		 * or return -ENOENT for an empty entry.
		 */
		rc = -ENOENT;
		if (cur_index == index) {
			if (!phandle)
				goto err;

			if (out_args) {
				int i;
				if (WARN_ON(count > MAX_PHANDLE_ARGS))
					count = MAX_PHANDLE_ARGS;
				out_args->np = node;
				out_args->args_count = count;
				for (i = 0; i < count; i++)
					out_args->args[i] =
						be32_to_cpup(list++);
			}

			/* Found it! return success */
			return 0;
		}

		node = NULL;
		list += count;
		cur_index++;
	}

	/*
	 * Unlock node before returning result; will be one of:
	 * -ENOENT : index is for empty phandle
	 * -EINVAL : parsing error on data
	 * [1..n]  : Number of phandle (count mode; when index = -1)
	 */
	rc = index < 0 ? cur_index : -ENOENT;
 err:
	return rc;
}

int of_parse_phandle_with_args(const struct device_node *np,
		const char *list_name, const char *cells_name, int index,
		struct of_phandle_args *out_args)
{
	if (index < 0)
		return -EINVAL;
	return __of_parse_phandle_with_args(np, list_name, cells_name,
					index, out_args);
}
EXPORT_SYMBOL(of_parse_phandle_with_args);

/**
 * of_count_phandle_with_args() - Find the number of phandles references in a property
 * @np:		pointer to a device tree node containing a list
 * @list_name:	property name that contains a list
 * @cells_name:	property name that specifies phandles' arguments count
 *
 * Returns the number of phandle + argument tuples within a property. It
 * is a typical pattern to encode a list of phandle and variable
 * arguments into a single property. The number of arguments is encoded
 * by a property in the phandle-target node. For example, a gpios
 * property would contain a list of GPIO specifies consisting of a
 * phandle and 1 or more arguments. The number of arguments are
 * determined by the #gpio-cells property in the node pointed to by the
 * phandle.
 */
int of_count_phandle_with_args(const struct device_node *np,
			const char *list_name, const char *cells_name)
{
	return __of_parse_phandle_with_args(np, list_name, cells_name,
					-1, NULL);
}
EXPORT_SYMBOL(of_count_phandle_with_args);

/**
 * of_machine_is_compatible - Test root of device tree for a given compatible value
 * @compat: compatible string to look for in root node's compatible property.
 *
 * Returns true if the root node has the given value in its
 * compatible property.
 */
int of_machine_is_compatible(const char *compat)
{
	if (!root_node)
		return 0;

	return of_device_is_compatible(root_node, compat);
}
EXPORT_SYMBOL(of_machine_is_compatible);

/**
 *	of_find_node_by_path_from - Find a node matching a full OF path
 *      relative to a given root node.
 *	@path:	The full path to match
 *
 *	Returns a pointer to the node found or NULL.
 */
struct device_node *of_find_node_by_path_from(struct device_node *from,
					const char *path)
{
	char *slash, *p, *freep;

	if (!from || !path || *path != '/')
		return NULL;

	path++;

	freep = p = xstrdup(path);

	while (1) {
		if (!*p)
			goto out;

		slash = strchr(p, '/');
		if (slash)
			*slash = 0;

		from = of_get_child_by_name(from, p);
		if (!from)
			goto out;

		if (!slash)
			goto out;

		p = slash + 1;
	}
out:
	free(freep);

	return from;
}
EXPORT_SYMBOL(of_find_node_by_path_from);

/**
 *	of_find_node_by_path - Find a node matching a full OF path
 *	@path:	The full path to match
 *
 *	Returns a pointer to the node found or NULL.
 */
struct device_node *of_find_node_by_path(const char *path)
{
	return of_find_node_by_path_from(root_node, path);
}
EXPORT_SYMBOL(of_find_node_by_path);

/**
 * of_modalias_node - Lookup appropriate modalias for a device node
 * @node:	pointer to a device tree node
 * @modalias:	Pointer to buffer that modalias value will be copied into
 * @len:	Length of modalias value
 *
 * Based on the value of the compatible property, this routine will attempt
 * to choose an appropriate modalias value for a particular device tree node.
 * It does this by stripping the manufacturer prefix (as delimited by a ',')
 * from the first entry in the compatible list property.
 *
 * This routine returns 0 on success, <0 on failure.
 */
int of_modalias_node(struct device_node *node, char *modalias, int len)
{
	const char *compatible, *p;
	int cplen;

	compatible = of_get_property(node, "compatible", &cplen);
	if (!compatible || strlen(compatible) > cplen)
		return -ENODEV;
	p = strchr(compatible, ',');
	strlcpy(modalias, p ? p + 1 : compatible, len);
	return 0;
}
EXPORT_SYMBOL_GPL(of_modalias_node);

struct device_node *of_get_root_node(void)
{
	return root_node;
}

int of_set_root_node(struct device_node *node)
{
	if (node && root_node)
		return -EBUSY;

	root_node = node;

	of_alias_scan();

	return 0;
}

/**
 *  of_device_is_available - check if a device is available for use
 *
 *  @device: Node to check for availability
 *
 *  Returns 1 if the status property is absent or set to "okay" or "ok",
 *  0 otherwise
 */
int of_device_is_available(const struct device_node *device)
{
	const char *status;
	int statlen;

	status = of_get_property(device, "status", &statlen);
	if (status == NULL)
		return 1;

	if (statlen > 0) {
		if (!strcmp(status, "okay") || !strcmp(status, "ok"))
			return 1;
	}

	return 0;
}
EXPORT_SYMBOL(of_device_is_available);

/**
 *	of_get_parent - Get a node's parent if any
 *	@node:	Node to get parent
 *
 *	Returns a pointer to the parent node or NULL if already at root.
 */
struct device_node *of_get_parent(const struct device_node *node)
{
	return (!node) ? NULL : node->parent;
}
EXPORT_SYMBOL(of_get_parent);

/**
 *	of_get_next_available_child - Find the next available child node
 *	@node:	parent node
 *	@prev:	previous child of the parent node, or NULL to get first
 *
 *      This function is like of_get_next_child(), except that it
 *      automatically skips any disabled nodes (i.e. status = "disabled").
 */
struct device_node *of_get_next_available_child(const struct device_node *node,
	struct device_node *prev)
{
	prev = list_prepare_entry(prev, &node->children, parent_list);
	list_for_each_entry_continue(prev, &node->children, parent_list)
		if (of_device_is_available(prev))
			return prev;
	return NULL;
}
EXPORT_SYMBOL(of_get_next_available_child);

/**
 *	of_get_child_count - Count child nodes of given parent node
 *	@parent:	parent node
 *
 *      Returns the number of child nodes or -EINVAL on NULL parent node.
 */
int of_get_child_count(const struct device_node *parent)
{
	struct device_node *child;
	int num = 0;

	if (!parent)
		return -EINVAL;

	for_each_child_of_node(parent, child)
		num++;

	return num;
}
EXPORT_SYMBOL(of_get_child_count);

/**
 *	of_get_available_child_count - Count available child nodes of given
 *      parent node
 *	@parent:	parent node
 *
 *      Returns the number of available child nodes or -EINVAL on NULL parent
 *      node.
 */
int of_get_available_child_count(const struct device_node *parent)
{
	struct device_node *child;
	int num = 0;

	if (!parent)
		return -EINVAL;

	for_each_child_of_node(parent, child)
		if (of_device_is_available(child))
			num++;

	return num;
}
EXPORT_SYMBOL(of_get_available_child_count);

/**
 *	of_get_child_by_name - Find the child node by name for a given parent
 *	@node:	parent node
 *	@name:	child name to look for.
 *
 *      This function looks for child node for given matching name
 *
 *	Returns a node pointer if found or NULL.
 */
struct device_node *of_get_child_by_name(const struct device_node *node,
				const char *name)
{
	struct device_node *child;

	for_each_child_of_node(node, child)
		if (child->name && (of_node_cmp(child->name, name) == 0))
			return child;

	return NULL;
}
EXPORT_SYMBOL(of_get_child_by_name);

void of_print_nodes(struct device_node *node, int indent)
{
	struct device_node *n;
	struct property *p;
	int i;

	if (!node)
		return;

	for (i = 0; i < indent; i++)
		printf("\t");

	printf("%s%s\n", node->name, node->name ? " {" : "{");

	list_for_each_entry(p, &node->properties, list) {
		for (i = 0; i < indent + 1; i++)
			printf("\t");
		printf("%s", p->name);
		if (p->length) {
			printf(" = ");
			of_print_property(p->value, p->length);
		}
		printf(";\n");
	}

	list_for_each_entry(n, &node->children, parent_list) {
		of_print_nodes(n, indent + 1);
	}

	for (i = 0; i < indent; i++)
		printf("\t");
	printf("};\n");
}

struct device_node *of_new_node(struct device_node *parent, const char *name)
{
	struct device_node *node;

	node = xzalloc(sizeof(*node));
	node->parent = parent;
	if (parent)
		list_add_tail(&node->parent_list, &parent->children);

	INIT_LIST_HEAD(&node->children);
	INIT_LIST_HEAD(&node->properties);

	if (parent) {
		node->name = xstrdup(name);
		node->full_name = asprintf("%s/%s", node->parent->full_name, name);
		list_add(&node->list, &parent->list);
	} else {
		node->name = xstrdup("");
		node->full_name = xstrdup("");
		INIT_LIST_HEAD(&node->list);
	}

	return node;
}

struct property *of_new_property(struct device_node *node, const char *name,
		const void *data, int len)
{
	struct property *prop;

	prop = xzalloc(sizeof(*prop));
	prop->name = strdup(name);
	if (!prop->name) {
		free(prop);
		return NULL;
	}

	prop->length = len;
	prop->value = xzalloc(len);

	if (data)
		memcpy(prop->value, data, len);

	list_add_tail(&prop->list, &node->properties);

	return prop;
}

void of_delete_property(struct property *pp)
{
	if (!pp)
		return;

	list_del(&pp->list);

	free(pp->name);
	free(pp->value);
	free(pp);
}

/**
 * of_set_property - create a property for a given node
 * @node - the node
 * @name - the name of the property
 * @val - the value for the property
 * @len - the length of the properties value
 * @create - if true, the property is created if not existing already
 */
int of_set_property(struct device_node *np, const char *name, const void *val, int len,
		int create)
{
	struct property *pp = of_find_property(np, name, NULL);

	if (!np)
		return -ENOENT;

	if (!pp && !create)
		return -ENOENT;

	of_delete_property(pp);

	pp = of_new_property(np, name, val, len);
	if (!pp)
		return -ENOMEM;

	return 0;
}

static struct device_d *add_of_amba_device(struct device_node *node)
{
	struct amba_device *dev;
	char *name, *at;

	dev = xzalloc(sizeof(*dev));

	name = xstrdup(node->name);
	at = strchr(name, '@');
	if (at) {
		*at = 0;
		snprintf(dev->dev.name, MAX_DRIVER_NAME, "%s.%s", at + 1, name);
	} else {
		strncpy(dev->dev.name, node->name, MAX_DRIVER_NAME);
	}

	dev->dev.id = DEVICE_ID_SINGLE;
	memcpy(&dev->res, &node->resource[0], sizeof(struct resource));
	dev->dev.resource = node->resource;
	dev->dev.num_resources = 1;
	dev->dev.device_node = node;
	node->device = &dev->dev;

	of_property_read_u32(node, "arm,primecell-periphid", &dev->periphid);

	debug("register device 0x%08x\n", node->resource[0].start);

	amba_device_add(dev);

	free(name);

	return &dev->dev;
}

static struct device_d *add_of_platform_device(struct device_node *node,
		struct device_d *parent)
{
	struct device_d *dev;
	char *name, *at;

	dev = xzalloc(sizeof(*dev));

	dev->parent = parent;

	name = xstrdup(node->name);
	at = strchr(name, '@');
	if (at) {
		*at = 0;
		snprintf(dev->name, MAX_DRIVER_NAME, "%s.%s", at + 1, name);
	} else {
		strncpy(dev->name, node->name, MAX_DRIVER_NAME);
	}

	dev->id = DEVICE_ID_SINGLE;
	dev->resource = node->resource;
	dev->num_resources = node->num_resource;
	dev->device_node = node;
	node->device = dev;

	debug("register device 0x%08x\n", node->resource[0].start);

	platform_device_register(dev);

	free(name);

	return dev;
}

static struct device_d *add_of_device(struct device_node *node,
		struct device_d *parent)
{
	const struct property *cp;

	if (!of_device_is_available(node))
		return NULL;

	cp = of_get_property(node, "compatible", NULL);
	if (!cp)
		return NULL;

	if (IS_ENABLED(CONFIG_ARM_AMBA) &&
	    of_device_is_compatible(node, "arm,primecell") == 1)
		return add_of_amba_device(node);
	else
		return add_of_platform_device(node, parent);
}
EXPORT_SYMBOL(add_of_device);

static u64 dt_mem_next_cell(int s, const __be32 **cellp)
{
	const __be32 *p = *cellp;

	*cellp = p + s;
	return of_read_number(p, s);
}

int of_add_memory(struct device_node *node, bool dump)
{
	int na, nc;
	const __be32 *reg, *endp;
	int len, r = 0, ret;
	const char *device_type;

	ret = of_property_read_string(node, "device_type", &device_type);
	if (ret)
		return -ENXIO;

	if (of_node_cmp(device_type, "memory"))
		return -ENXIO;

	of_bus_count_cells(node, &na, &nc);

	reg = of_get_property(node, "reg", &len);
	if (!reg)
		return -EINVAL;

	endp = reg + (len / sizeof(__be32));

	while ((endp - reg) >= (na + nc)) {
		u64 base, size;

		base = dt_mem_next_cell(na, &reg);
		size = dt_mem_next_cell(nc, &reg);

		if (size == 0)
			continue;

		of_add_memory_bank(node, dump, r, base, size);

		r++;
	}

	return 0;
}

static struct device_d *add_of_device_resource(struct device_node *node,
		struct device_d *parent)
{
	u64 address = 0, size;
	struct resource *res, *resp;
	struct device_d *dev;
	const __be32 *endp, *reg;
	const char *resname;
	int na, nc, n_resources;
	int ret, len, index;

	reg = of_get_property(node, "reg", &len);
	if (!reg)
		return add_of_device(node, parent);

	of_bus_count_cells(node, &na, &nc);

	n_resources = (len / sizeof(__be32)) / (na + nc);

	res = resp = xzalloc(sizeof(*res) * n_resources);

	endp = reg + (len / sizeof(__be32));

	index = 0;

	while ((endp - reg) >= (na + nc)) {
		address = of_translate_address(node, reg);
		if (address == OF_BAD_ADDR) {
			ret =  -EINVAL;
			goto err_free;
		}

		reg += na;
		size = dt_mem_next_cell(nc, &reg);

		resp->start = address;
		resp->end = address + size - 1;
		resname = NULL;
		of_property_read_string_index(node, "reg-names", index, &resname);
		if (resname)
			resp->name = xstrdup(resname);
		resp->flags = IORESOURCE_MEM;
		resp++;
		index++;
        }

	/*
	 * A device may already be registered as platform_device.
	 * Instead of registering the same device again, just
	 * add this node to the existing device.
	 */
	for_each_device(dev) {
		if (!dev->resource)
			continue;
		if (dev->resource->start == res->start &&
				dev->resource->end == res->end) {
			debug("connecting %s to %s\n", node->name, dev_name(dev));
			node->device = dev;
			dev->device_node = node;
			node->resource = dev->resource;
			ret = 0;
			goto err_free;
		}
	}

	node->resource = res;
	node->num_resource = n_resources;

	return add_of_device(node, parent);

err_free:
	free(res);

	return NULL;
}

void of_free(struct device_node *node)
{
	struct device_node *n, *nt;
	struct property *p, *pt;

	if (!node)
		return;

	list_for_each_entry_safe(p, pt, &node->properties, list)
		of_delete_property(p);

	list_for_each_entry_safe(n, nt, &node->children, parent_list) {
		of_free(n);
	}

	if (node->parent) {
		list_del(&node->parent_list);
		list_del(&node->list);
	}

	if (node->device)
		node->device->device_node = NULL;
	else
		free(node->resource);

	free(node->name);
	free(node->full_name);
	free(node);

	if (node == root_node)
		of_set_root_node(NULL);
}

static void __of_probe(struct device_node *node,
		const struct of_device_id *matches,
		struct device_d *parent)
{
	struct device_node *n;
	struct device_d *dev;

	if (node->device)
		return;

	dev = add_of_device_resource(node, parent);

	if (!of_match_node(matches, node))
		return;

	list_for_each_entry(n, &node->children, parent_list)
		__of_probe(n, matches, dev);
}

static void __of_parse_phandles(struct device_node *node)
{
	struct device_node *n;
	phandle phandle;
	int ret;

	ret = of_property_read_u32(node, "phandle", &phandle);
	if (!ret) {
		node->phandle = phandle;
		list_add_tail(&node->phandles, &phandle_list);
	}

	list_for_each_entry(n, &node->children, parent_list)
		__of_parse_phandles(n);
}

struct device_node *of_chosen;
const char *of_model;

const char *of_get_model(void)
{
	return of_model;
}

const struct of_device_id of_default_bus_match_table[] = {
	{
		.compatible = "simple-bus",
	}, {
		/* sentinel */
	}
};

int of_probe(void)
{
	struct device_node *memory, *n;

	if(!root_node)
		return -ENODEV;

	of_chosen = of_find_node_by_path("/chosen");
	of_property_read_string(root_node, "model", &of_model);

	__of_parse_phandles(root_node);

	memory = of_find_node_by_path("/memory");
	if (memory)
		of_add_memory(memory, false);

	list_for_each_entry(n, &root_node->children, parent_list)
		__of_probe(n, of_default_bus_match_table, NULL);

	return 0;
}

/**
 * of_create_node - create a new node including its parents
 * @path - the nodepath to create
 */
struct device_node *of_create_node(struct device_node *root, const char *path)
{
	char *slash, *p, *freep;
	struct device_node *tmp, *dn = root;

	if (*path != '/')
		return NULL;

	path++;

	p = freep = xstrdup(path);

	while (1) {
		if (!*p)
			goto out;

		slash = strchr(p, '/');
		if (slash)
			*slash = 0;

		tmp = of_get_child_by_name(dn, p);
		if (tmp)
			dn = tmp;
		else
			dn = of_new_node(dn, p);

		if (!dn)
			goto out;

		if (!slash)
			goto out;

		p = slash + 1;
	}
out:
	free(freep);

	return dn;
}

int of_device_is_stdout_path(struct device_d *dev)
{
	struct device_node *dn;
	const char *name;

	name = of_get_property(of_chosen, "linux,stdout-path", NULL);
	if (name == NULL)
		return 0;

	dn = of_find_node_by_path(name);
	if (!dn)
		return 0;

	if (dn == dev->device_node)
		return 1;

	return 0;
}

/**
 * of_add_initrd - add initrd properties to the devicetree
 * @root - the root node of the tree
 * @start - physical start address of the initrd image
 * @end - physical end address of the initrd image
 *
 * Add initrd properties to the devicetree, or, if end is 0,
 * delete them.
 *
 * Note that Linux interprets end differently than barebox. For Linux end points
 * to the first address after the memory occupied by the image while barebox
 * lets end pointing to the last occupied byte.
 */
int of_add_initrd(struct device_node *root, resource_size_t start,
		resource_size_t end)
{
	struct device_node *chosen;
	__be32 buf[2];

	chosen = of_find_node_by_path_from(root, "/chosen");
	if (!chosen)
		return -EINVAL;

	if (end) {
		of_write_number(buf, start, 2);
		of_set_property(chosen, "linux,initrd-start", buf, 8, 1);
		of_write_number(buf, end + 1, 2);
		of_set_property(chosen, "linux,initrd-end", buf, 8, 1);
	} else {
		struct property *pp;

		pp = of_find_property(chosen, "linux,initrd-start", NULL);
		if (pp)
			of_delete_property(pp);

		pp = of_find_property(chosen, "linux,initrd-end", NULL);
		if (pp)
			of_delete_property(pp);
	}

	return 0;
}
