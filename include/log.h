
#ifndef _LOG_H_
#define _LOG_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef enum LOG_LEVEL
{
  LOG_LVL_DEBUG,
  LOG_LVL_INFO,
  LOG_LVL_WARN,
  LOG_LVL_ERROR,
  LOG_LVL_FATAL,
  LOG_LVL_NONE
} LOG_LEVEL;


LOG_LEVEL log_get_level(void);

void log_set_level(LOG_LEVEL level);

void log_msg(LOG_LEVEL level, const char* format, ...);
//void log_msg(LOG_LEVEL level, ...);

#define LOG_D(...) log_msg(LOG_LVL_DEBUG, __VA_ARGS__)
#define LOG_I(...) log_msg(LOG_LVL_INFO , __VA_ARGS__)
#define LOG_W(...) log_msg(LOG_LVL_WARN , __VA_ARGS__)
#define LOG_E(...) log_msg(LOG_LVL_ERROR, __VA_ARGS__)
#define LOG_F(...) log_msg(LOG_LVL_FATAL, __VA_ARGS__)


#ifdef __cplusplus
}
#endif

#endif /*_LOG_H_*/
