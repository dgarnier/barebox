#ifndef __OF_H
#define __OF_H

#include <fdt.h>
#include <errno.h>
#include <asm/byteorder.h>

/* Default string compare functions */
#define of_compat_cmp(s1, s2, l)	strcasecmp((s1), (s2))
#define of_prop_cmp(s1, s2)		strcmp((s1), (s2))
#define of_node_cmp(s1, s2)		strcasecmp((s1), (s2))

#define OF_BAD_ADDR      ((u64)-1)

typedef u32 phandle;

struct property {
	char *name;
	int length;
	void *value;
	struct list_head list;
};

struct device_node {
	char *name;
	char *full_name;

	struct list_head properties;
	struct device_node *parent;
	struct list_head children;
	struct list_head parent_list;
	struct list_head list;
	struct resource *resource;
	int num_resource;
	struct device_d *device;
	struct list_head phandles;
	phandle phandle;
};

struct of_device_id {
	char *compatible;
	unsigned long data;
};

#define MAX_PHANDLE_ARGS 8
struct of_phandle_args {
	struct device_node *np;
	int args_count;
	uint32_t args[MAX_PHANDLE_ARGS];
};

#define OF_MAX_RESERVE_MAP	16
struct of_reserve_map {
	uint64_t start[OF_MAX_RESERVE_MAP];
	uint64_t end[OF_MAX_RESERVE_MAP];
	int num_entries;
};

int of_add_reserve_entry(resource_size_t start, resource_size_t end);
struct of_reserve_map *of_get_reserve_map(void);
void of_clean_reserve_map(void);
void fdt_add_reserve_map(void *fdt);

struct driver_d;

int of_fix_tree(struct device_node *);
int of_register_fixup(int (*fixup)(struct device_node *));

int of_match(struct device_d *dev, struct driver_d *drv);

int of_add_initrd(struct device_node *root, resource_size_t start,
		resource_size_t end);

struct fdt_header *fdt_get_tree(void);

struct fdt_header *of_get_fixed_tree(struct device_node *node);

/* Helper to read a big number; size is in cells (not bytes) */
static inline u64 of_read_number(const __be32 *cell, int size)
{
	u64 r = 0;
	while (size--)
		r = (r << 32) | be32_to_cpu(*(cell++));
	return r;
}

/* Helper to write a big number; size is in cells (not bytes) */
static inline void of_write_number(void *__cell, u64 val, int size)
{
	__be32 *cell = __cell;

	while (size--) {
		cell[size] = cpu_to_be32(val);
		val >>= 32;
	}
}

int of_get_named_gpio(struct device_node *np,
                                   const char *propname, int index);

void of_print_property(const void *data, int len);
void of_print_cmdline(struct device_node *root);

u64 of_translate_address(struct device_node *node, const __be32 *in_addr);

#define OF_ROOT_NODE_SIZE_CELLS_DEFAULT 1
#define OF_ROOT_NODE_ADDR_CELLS_DEFAULT 1

void of_print_nodes(struct device_node *node, int indent);
int of_probe(void);
int of_parse_dtb(struct fdt_header *fdt);
struct device_node *of_unflatten_dtb(struct device_node *root, void *fdt);

struct cdev;

#ifdef CONFIG_OFTREE
extern int of_n_addr_cells(struct device_node *np);
extern int of_n_size_cells(struct device_node *np);

extern struct property *of_find_property(const struct device_node *np,
					const char *name, int *lenp);
extern const void *of_get_property(const struct device_node *np,
				const char *name, int *lenp);

extern int of_set_property(struct device_node *node, const char *p,
			const void *val, int len, int create);
extern struct property *of_new_property(struct device_node *node,
				const char *name, const void *data, int len);
extern void of_delete_property(struct property *pp);

extern struct device_node *of_find_node_by_name(struct device_node *from,
	const char *name);
extern struct device_node *of_find_node_by_path_from(struct device_node *from,
						const char *path);
extern struct device_node *of_find_node_by_path(const char *path);
extern struct device_node *of_find_node_by_phandle(phandle phandle);
extern struct device_node *of_find_compatible_node(struct device_node *from,
	const char *type, const char *compat);
extern const struct of_device_id *of_match_node(
	const struct of_device_id *matches, const struct device_node *node);
extern struct device_node *of_find_matching_node_and_match(
	struct device_node *from,
	const struct of_device_id *matches,
	const struct of_device_id **match);
extern struct device_node *of_find_node_with_property(
	struct device_node *from, const char *prop_name);

extern struct device_node *of_new_node(struct device_node *parent,
				const char *name);
extern struct device_node *of_create_node(struct device_node *root,
					const char *path);
extern void of_free(struct device_node *node);

extern int of_machine_is_compatible(const char *compat);
extern int of_device_is_compatible(const struct device_node *device,
		const char *compat);
extern int of_device_is_available(const struct device_node *device);

extern struct device_node *of_get_parent(const struct device_node *node);
extern struct device_node *of_get_next_available_child(
	const struct device_node *node, struct device_node *prev);
extern int of_get_child_count(const struct device_node *parent);
extern int of_get_available_child_count(const struct device_node *parent);
extern struct device_node *of_get_child_by_name(const struct device_node *node,
					const char *name);

extern int of_property_read_u32_index(const struct device_node *np,
				       const char *propname,
				       u32 index, u32 *out_value);
extern int of_property_read_u8_array(const struct device_node *np,
			const char *propname, u8 *out_values, size_t sz);
extern int of_property_read_u16_array(const struct device_node *np,
			const char *propname, u16 *out_values, size_t sz);
extern int of_property_read_u32_array(const struct device_node *np,
				      const char *propname,
				      u32 *out_values,
				      size_t sz);
extern int of_property_read_u64(const struct device_node *np,
				const char *propname, u64 *out_value);

extern int of_property_read_string(struct device_node *np,
				   const char *propname,
				   const char **out_string);
extern int of_property_read_string_index(struct device_node *np,
					 const char *propname,
					 int index, const char **output);
extern int of_property_match_string(struct device_node *np,
				    const char *propname,
				    const char *string);
extern int of_property_count_strings(struct device_node *np,
				     const char *propname);

extern const __be32 *of_prop_next_u32(struct property *prop,
				const __be32 *cur, u32 *pu);
extern const char *of_prop_next_string(struct property *prop, const char *cur);

extern int of_property_write_bool(struct device_node *np,
				const char *propname, const bool value);
extern int of_property_write_u8_array(struct device_node *np,
				const char *propname, const u8 *values,
				size_t sz);
extern int of_property_write_u16_array(struct device_node *np,
				const char *propname, const u16 *values,
				size_t sz);
extern int of_property_write_u32_array(struct device_node *np,
				const char *propname, const u32 *values,
				size_t sz);
extern int of_property_write_u64_array(struct device_node *np,
				const char *propname, const u64 *values,
				size_t sz);

extern struct device_node *of_parse_phandle(const struct device_node *np,
					    const char *phandle_name,
					    int index);
extern int of_parse_phandle_with_args(const struct device_node *np,
	const char *list_name, const char *cells_name, int index,
	struct of_phandle_args *out_args);
extern int of_count_phandle_with_args(const struct device_node *np,
	const char *list_name, const char *cells_name);

extern void of_alias_scan(void);
extern int of_alias_get_id(struct device_node *np, const char *stem);
extern const char *of_alias_get(struct device_node *np);
extern int of_modalias_node(struct device_node *node, char *modalias, int len);

extern struct device_node *of_get_root_node(void);
extern int of_set_root_node(struct device_node *node);

int of_parse_partitions(struct cdev *cdev, struct device_node *node);
int of_device_is_stdout_path(struct device_d *dev);
const char *of_get_model(void);
void *of_flatten_dtb(struct device_node *node);
int of_add_memory(struct device_node *node, bool dump);
void of_add_memory_bank(struct device_node *node, bool dump, int r,
		u64 base, u64 size);
#else
static inline int of_parse_partitions(struct cdev *cdev,
					  struct device_node *node)
{
	return -EINVAL;
}

static inline int of_device_is_stdout_path(struct device_d *dev)
{
	return 0;
}

static inline const char *of_get_model(void)
{
	return NULL;
}

static inline void *of_flatten_dtb(struct device_node *node)
{
	return NULL;
}

static inline int of_add_memory(struct device_node *node, bool dump)
{
	return -EINVAL;
}

static inline struct device_node *of_get_root_node(void)
{
	return NULL;
}

static inline int of_set_root_node(struct device_node *node)
{
	return -ENOSYS;
}

static inline int of_n_addr_cells(struct device_node *np)
{
	return 0;
}

static inline int of_n_size_cells(struct device_node *np)
{
	return 0;
}

static inline struct device_node *of_get_parent(const struct device_node *node)
{
	return NULL;
}

static inline struct device_node *of_get_next_available_child(
		const struct device_node *node, struct device_node *prev)
{
	return NULL;
}

static inline int of_get_child_count(const struct device_node *parent)
{
	return -ENOSYS;
}

static inline int of_get_available_child_count(const struct device_node *parent)
{
	return -ENOSYS;
}

static inline struct device_node *of_get_child_by_name(
			const struct device_node *node, const char *name)
{
	return NULL;
}

static inline struct property *of_find_property(const struct device_node *np,
						const char *name,
						int *lenp)
{
	return NULL;
}

static inline const void *of_get_property(const struct device_node *np,
				const char *name, int *lenp)
{
	return NULL;
}

static inline int of_set_property(struct device_node *node, const char *p,
			const void *val, int len, int create)
{
	return -ENOSYS;
}

static inline struct property *of_new_property(struct device_node *node,
				const char *name, const void *data, int len)
{
	return NULL;
}

static inline void of_delete_property(struct property *pp)
{
}

static inline int of_property_read_u32_index(const struct device_node *np,
				const char *propname, u32 index, u32 *out_value)
{
	return -ENOSYS;
}

static inline int of_property_read_u8_array(const struct device_node *np,
				const char *propname, u8 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_u16_array(const struct device_node *np,
			const char *propname, u16 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_u32_array(const struct device_node *np,
			const char *propname, u32 *out_values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_read_u64(const struct device_node *np,
				const char *propname, u64 *out_value)
{
	return -ENOSYS;
}

static inline int of_property_read_string(struct device_node *np,
				const char *propname, const char **out_string)
{
	return -ENOSYS;
}

static inline int of_property_read_string_index(struct device_node *np,
			 const char *propname, int index, const char **output)
{
	return -ENOSYS;
}

static inline int of_property_match_string(struct device_node *np,
				const char *propname, const char *string)
{
	return -ENOSYS;
}

static inline int of_property_count_strings(struct device_node *np,
					const char *propname)
{
	return -ENOSYS;
}

static inline const __be32 *of_prop_next_u32(struct property *prop,
					const __be32 *cur, u32 *pu)
{
	return 0;
}

static inline const char *of_prop_next_string(struct property *prop,
					const char *cur)
{
	return NULL;
}

static inline int of_property_write_bool(struct device_node *np,
				const char *propname, const bool value)
{
	return -ENOSYS;
}

static inline int of_property_write_u8_array(struct device_node *np,
			const char *propname, const u8 *values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_write_u16_array(struct device_node *np,
			const char *propname, const u16 *values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_write_u32_array(struct device_node *np,
			const char *propname, const u32 *values, size_t sz)
{
	return -ENOSYS;
}

static inline int of_property_write_u64_array(struct device_node *np,
			const char *propname, const u64 *values, size_t sz)
{
	return -ENOSYS;
}

static inline struct device_node *of_parse_phandle(const struct device_node *np,
					    const char *phandle_name, int index)
{
	return NULL;
}

static inline int of_parse_phandle_with_args(const struct device_node *np,
		const char *list_name, const char *cells_name, int index,
		struct of_phandle_args *out_args)
{
	return -ENOSYS;
}

static inline int of_count_phandle_with_args(const struct device_node *np,
				const char *list_name, const char *cells_name)
{
	return -ENOSYS;
}

static inline struct device_node *of_find_node_by_path_from(
	struct device_node *from, const char *path)
{
	return NULL;
}

static inline struct device_node *of_find_node_by_path(const char *path)
{
	return NULL;
}

static inline struct device_node *of_find_node_by_name(struct device_node *from,
	const char *name)
{
	return NULL;
}

static inline struct device_node *of_find_node_by_phandle(phandle phandle)
{
	return NULL;
}

static inline struct device_node *of_find_compatible_node(
						struct device_node *from,
						const char *type,
						const char *compat)
{
	return NULL;
}

static inline const struct of_device_id *of_match_node(
	const struct of_device_id *matches, const struct device_node *node)
{
	return NULL;
}

static inline struct device_node *of_find_matching_node_and_match(
					struct device_node *from,
					const struct of_device_id *matches,
					const struct of_device_id **match)
{
	return NULL;
}

static inline struct device_node *of_find_node_with_property(
			struct device_node *from, const char *prop_name)
{
	return NULL;
}

static inline struct device_node *of_new_node(struct device_node *parent,
				const char *name)
{
	return NULL;
}

static inline struct device_node *of_create_node(struct device_node *root,
					const char *path)
{
	return NULL;
}

static inline void of_free(struct device_node *node)
{
}

static inline int of_machine_is_compatible(const char *compat)
{
	return 0;
}

static inline int of_device_is_compatible(const struct device_node *device,
		const char *compat)
{
	return 0;
}

static inline int of_device_is_available(const struct device_node *device)
{
	return 0;
}

static inline void of_alias_scan(void)
{
}

static inline int of_alias_get_id(struct device_node *np, const char *stem)
{
	return -ENOSYS;
}

static inline const char *of_alias_get(struct device_node *np)
{
	return NULL;
}

static inline int of_modalias_node(struct device_node *node, char *modalias,
				int len)
{
	return -ENOSYS;
}
#endif

#define for_each_node_by_name(dn, name) \
	for (dn = of_find_node_by_name(NULL, name); dn; \
	     dn = of_find_node_by_name(dn, name))
#define for_each_compatible_node(dn, type, compatible) \
	for (dn = of_find_compatible_node(NULL, type, compatible); dn; \
	     dn = of_find_compatible_node(dn, type, compatible))
static inline struct device_node *of_find_matching_node(
	struct device_node *from,
	const struct of_device_id *matches)
{
	return of_find_matching_node_and_match(from, matches, NULL);
}
#define for_each_matching_node(dn, matches) \
	for (dn = of_find_matching_node(NULL, matches); dn; \
	     dn = of_find_matching_node(dn, matches))
#define for_each_matching_node_and_match(dn, matches, match) \
	for (dn = of_find_matching_node_and_match(NULL, matches, match); \
	     dn; dn = of_find_matching_node_and_match(dn, matches, match))
#define for_each_node_with_property(dn, prop_name) \
	for (dn = of_find_node_with_property(NULL, prop_name); dn; \
	     dn = of_find_node_with_property(dn, prop_name))

#define for_each_child_of_node(parent, child) \
	list_for_each_entry(child, &parent->children, parent_list)
#define for_each_available_child_of_node(parent, child) \
	for (child = of_get_next_available_child(parent, NULL); child != NULL; \
	     child = of_get_next_available_child(parent, child))

/**
 * of_property_read_bool - Findfrom a property
 * @np:		device node from which the property value is to be read.
 * @propname:	name of the property to be searched.
 *
 * Search for a property in a device node.
 * Returns true if the property exist false otherwise.
 */
static inline bool of_property_read_bool(const struct device_node *np,
					 const char *propname)
{
	struct property *prop = of_find_property(np, propname, NULL);

	return prop ? true : false;
}

static inline int of_property_read_u8(const struct device_node *np,
				       const char *propname,
				       u8 *out_value)
{
	return of_property_read_u8_array(np, propname, out_value, 1);
}

static inline int of_property_read_u16(const struct device_node *np,
				       const char *propname,
				       u16 *out_value)
{
	return of_property_read_u16_array(np, propname, out_value, 1);
}

static inline int of_property_read_u32(const struct device_node *np,
				       const char *propname,
				       u32 *out_value)
{
	return of_property_read_u32_array(np, propname, out_value, 1);
}

/*
 * struct property *prop;
 * const __be32 *p;
 * u32 u;
 *
 * of_property_for_each_u32(np, "propname", prop, p, u)
 *         printk("U32 value: %x\n", u);
 */
#define of_property_for_each_u32(np, propname, prop, p, u)	\
	for (prop = of_find_property(np, propname, NULL),	\
		p = of_prop_next_u32(prop, NULL, &u);		\
		p;						\
		p = of_prop_next_u32(prop, p, &u))

/*
 * struct property *prop;
 * const char *s;
 *
 * of_property_for_each_string(np, "propname", prop, s)
 *         printk("String value: %s\n", s);
 */
#define of_property_for_each_string(np, propname, prop, s)	\
	for (prop = of_find_property(np, propname, NULL),	\
		s = of_prop_next_string(prop, NULL);		\
		s;						\
		s = of_prop_next_string(prop, s))

static inline int of_property_write_u8(struct device_node *np,
				       const char *propname, u8 value)
{
	return of_property_write_u8_array(np, propname, &value, 1);
}

static inline int of_property_write_u16(struct device_node *np,
					const char *propname, u16 value)
{
	return of_property_write_u16_array(np, propname, &value, 1);
}

static inline int of_property_write_u32(struct device_node *np,
					const char *propname,
					u32 value)
{
	return of_property_write_u32_array(np, propname, &value, 1);
}

static inline int of_property_write_u64(struct device_node *np,
					const char *propname,
					u64 value)
{
	return of_property_write_u64_array(np, propname, &value, 1);
}

#endif /* __OF_H */
