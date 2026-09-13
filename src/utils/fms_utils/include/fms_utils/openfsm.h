/*
# File    :   openfsm.h
# Author  :   schips
# Git     :   https://gitee.com/schips/
# Date    :   2019-05-27 
# Describe:   一个简单的fsm组件，开发者可以基于这个组件进行二次开发
*/

#ifndef __FSM_H_
#define __FSM_H_

#include <functional>
#include <iostream>

// sun: 这是一个与业务无关的轻量 FSM 容器。业务状态函数通过 Procedure 数组注册，
// sun: 每次 run_state_machine_once() 调用当前状态过程，并在周期边界更新状态编号。
/* ----------------------- Defines ------------------------------------------*/
// 用于快速识别出 STATE与STEP
#define FSM_STATE(name) state_##name
#define FSM_FUNCT(name) funct_##name

// 数据类型定义区
typedef unsigned char state;
typedef long long step_ret;
typedef void* AS_STEP_RETVAL;


/*!
 *  @brief  状态机过程实现原型函数
 *          设计状态机时需要按照这个模式写
 *
 *  @param[in]  void* 你所需要的任何参数
 *
 *  @return 返回值 代表下一个状态
 */
// typedef void* (*Procedure)(void *);
typedef std::function<void*(void*)> Procedure;

typedef struct {
    // sun: ds/ls/cs/ns 分别表示默认、上次、当前和下一状态；跳转通过修改 ns 延迟生效。
    state ds;       // 默认状态
    state ls;       // 上个状态
    state cs;       // 当前状态
    state ns;       // 下个状态
} SM_STATE;

// 状态机 属性 定义
typedef struct {
    // 状态管理
    SM_STATE st;

    // sun: procedures 由状态编号直接索引，因此调用方必须保证数组顺序与枚举一致。
    Procedure *procedures;

    // 状态机数据区域
    void * data;

    // 错误处理（用于存放 状态 执行 的结果）
    step_ret ret_ptr; // 状态 执行结果
    void * err_ptr;
    state err_flag;
} FSM;

/* ----------------------- Start function declaration -----------------------------*/

/*!
 *  @brief  设置状态机的错误容器
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @param[in]  err_var 容器
 *
 *  @return 是/否
 */
static inline void set_err_var(FSM* fsm, void* err_var)
{
    if(!fsm) return ;
    fsm->err_ptr = err_var;
}

/*!
 *  @brief  获取错误值容器（用于读取其中的内容）
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 是/否
 */
static inline void* get_err_var(FSM* fsm)
{
    return fsm->err_ptr;
}

/*!
 *  @brief  获取状态机 在 步进中是否遇到了错误。
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 是/否
 */
static inline state is_fsm_error(FSM* fsm)
{
    return fsm->err_flag;
}

/*!
 *  @brief  置 状态机 错误位
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 是/否
 */
static inline state set_fsm_error_flag(FSM* fsm)
{
    if(!fsm) return -1;
    fsm->err_flag = 1;
    return 0;
}

/*!
 *  @brief  置 状态机 错误位
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 是/否
 */
static inline state clr_fsm_error_flag(FSM* fsm)
{
    if(!fsm) return -1;
    fsm->err_flag = 0;
    return 0;
}


/*!
 *  @brief  为状态机添加 过程方法 序列
 *
 *  @param[in] fsm 状态机实例
 *
 *  @param[in] procedures 状态机的所有过程方法
 *
 */
static inline void set_procedures(FSM * fsm, Procedure *procedures)
{
    if(fsm)
    {
        fsm->procedures = procedures;
        fsm->st.cs = -1; // 执行run之前，当前状态是未定的
    }
}

/*!
 *  @brief  配置状态机的数据域
 *
 *  @param[in] fsm 状态机实例
 *
 *  @param[in] data 状态机需要的数据域
 *
 */
static inline void set_data_entry(FSM * fsm, void *data)
{
	if(fsm) fsm->data = data;
}

/*!
 *  @brief  配置状态机的数据域
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 返回 状态机 数据域
 *
 */
static inline void* get_data_entry(FSM * fsm)
{
    return fsm->data;
}

/*!
 *  @brief  让 状态机 步进一次
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 非负数 ：代表 所成功执行的状态
 *          -1 : 失败 
 */
static inline state run_state_machine_once(FSM * fsm)
{
    // sun: 一次步进只执行一个状态过程，过程可通过 set_next_state() 安排下一周期跳转。
    if(!fsm) return -1;

    // 切换到新状态
    fsm->st.cs = fsm->st.ns;

    // 跳转到下一个状态（状态 执行 结果 保存在 ret_ptr 中 ）
    fsm->ret_ptr = (step_ret)fsm->procedures[fsm->st.cs](fsm);

    return fsm->st.cs;
}

/*!
 *  @brief  获取步进执行结果
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 是/否
 */
static inline void* get_step_retval(FSM* fsm)
{
    return (void *)fsm->ret_ptr;
}

/*!
 *  @brief  获取状态机的当前状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 当前状态
 */
static inline state get_curr_state(FSM * fsm)
{
    return fsm->st.cs;
}

/*!
 *  @brief  设置状态机默认状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @param[in]  st  状态值
 *
 */
static inline void set_default_state(FSM * fsm, state st)
{
    if(!fsm) 
    {
        std::cout << "fsm is null" << std::endl;
        return;
    }
    
    fsm->st.ds = st;
    fsm->st.ls = st;
    fsm->st.ns = st;
}

/*!
 *  @brief  设置状态机的下次状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @param[in]  st  状态值
 *
 */
static inline void set_next_state(FSM * fsm, state st)
{
    if(fsm) fsm->st.ns = st;
}

/*!
 *  @brief  获取状态机的下次状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 下一个状态
 */
static inline state get_next_state(FSM * fsm)
{
    return fsm->st.ns;
}

/*!
 *  @brief  设置状态机的上次状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @param[in]  st  状态值
 *
 */
static inline void set_last_state(FSM * fsm)
{
    if(fsm) fsm->st.ls = fsm->st.cs;
}

/*!
 *  @brief  获取状态机的上次状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @return 上一个状态
 */
static inline state get_last_state(FSM * fsm)
{
    return fsm->st.ls;
}

/*!
 *  @brief  将状态机设为默认状态
 *
 *  @param[in]  fsm 状态机实例
 *
 */
static inline void init_state_machine(FSM *p)
{
    // sun: 初始化把下一状态指向默认状态，首次步进后 cs/ls 会按统一流程建立。
    set_next_state(p, p->st.ds);
    p->st.cs = -1; // 执行run之前，当前状态是未定的
}

/*!
 *  @brief  将状态机设为默认状态，同时清除错误状态
 *
 *  @param[in]  fsm 状态机实例
 *
 */
static inline void reset_state_machine(FSM *p)
{
    if(!p) return ;
    clr_fsm_error_flag(p);
    init_state_machine(p);
}

/*!
 *  @brief  判断状态机是否在某个状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @param[in]  st  状态值
 *
 *  @return 是/否
 */
static inline state is_curr_state(FSM* fsm, state st)
{
    return fsm->st.cs == st;
}

/*!
 *  @brief  判断状态机是否即将进行某个状态
 *
 *  @param[in]  fsm 状态机实例
 *
 *  @param[in]  st  状态值
 *
 *  @return 是/否
 */
static inline state is_next_state(FSM* fsm, state st)
{
    return fsm->st.ns == st;
}

#endif /* __FSM_H_ */
