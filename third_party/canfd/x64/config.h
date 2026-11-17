#ifndef ZLG_CONFIG_INTF_H
#define ZLG_CONFIG_INTF_H

struct _Meta;
struct _Pair;
struct _Options;
struct _ConfigNode;

typedef struct _Meta Meta;
typedef struct _Pair Pair;
typedef struct _Options Options;
typedef struct _ConfigNode ConfigNode;

/**
 * Optional value of a node meta.
 */
struct _Options
{
    /*! Option type */
    const char * type;

    /*! Option value */
    const char * value;

    /*! Option description */
    const char * desc;
};

/**
 * Meta information of a node.
 */
struct _Meta
{
    /*! Type of the node */
    const char * type;

    /*! Description of the node */
    const char * desc;

    /*! 1: read only, 0: read write, default is read write */
    int read_only;

    /*! Display format, for numeric nodes this is the printf style format */
    const char * format;

    /*! Minimum value, or minimum string length */
    double min_value;

    /*! Maximum value, or maximum string length */
    double max_value;

    /*! Unit of the value */
    const char * unit;

    /*! Step used by spin box/scroll bar style editors */
    double delta;

    /*! true: the node is visible, false: hidden */
    const char* visible;

    /*! true: the node is enabled, false: disabled */
    const char* enable;

    /*! Non zero if the value can be edited */
    int editable;

    /*! Optional values, terminated by NULL */
    Options** options;
};

/**
 * Key value pair of a node attribute.
 */
struct _Pair
{
    const char * key;
    const char * value;
};

/**
 * Configuration node.
 */
struct _ConfigNode
{
    /*! Node name */
    const char * name;

    /*! Node value */
    const char * value;

    /*! Formatted value of the node */
    const char* binding_value;

    /*! Full path of the node */
    const char * path;

    /*! Meta information */
    Meta* meta_info;

    /*! Children, terminated by NULL */
    ConfigNode** children;

    /*! Attributes, terminated by NULL */
    Pair** attributes;
};

/**
 * \brief Get the meta information of a property node.
 * \retval ConfigNode
 */
typedef const ConfigNode* (*GetPropertysFunc)();

/**
 * \brief Set the value of the property at the given path.
 * \param[in] path  : property path.
 * \param[in] value : property value.
 * \retval 1 on success, 0 on failure.
 */
typedef int (*SetValueFunc)(const char* path, const char* value);

/**
 * \brief Get the value of the property at the given path.
 * \param[in] path : property path.
 * \retval property value on success, NULL on failure.
 */
typedef const char* (*GetValueFunc)(const char* path);

typedef struct tagIProperty
{
    SetValueFunc     SetValue;
    GetValueFunc     GetValue;
    GetPropertysFunc GetPropertys;
}IProperty;

#endif/*ZLG_CONFIG_INTF_H */