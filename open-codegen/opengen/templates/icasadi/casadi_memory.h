/*
 * Header file with sizes of arrays that need to be allocated
 * (statically) once
 *
 *
 * This file is autogenerated by Optimization Engine
 * See http://doc.optimization-engine.xyz
 *
 *
 * Metadata:
 * + Optimizer
 *   + name: {{ meta.optimizer_name }}
 *   + version: {{ meta.version }}
 *   + licence: {{ meta.licence }}
 *
 *
 */

#pragma once

/*
 * Cost sizes
 */
#define COST_SZ_ARG_{{ meta.optimizer_name | upper}} {{ cost.sz_arg() }}
#define COST_SZ_IW_{{ meta.optimizer_name | upper}} {{ cost.sz_iw() }}
#define COST_SZ_W_{{ meta.optimizer_name | upper}} {{ cost.sz_w() }}
#define COST_SZ_RES_{{ meta.optimizer_name | upper}} {{ cost.sz_res() }}

/*
 * Gradient sizes
 */
#define GRAD_SZ_ARG_{{ meta.optimizer_name | upper}} {{ grad.sz_arg() }}
#define GRAD_SZ_IW_{{ meta.optimizer_name | upper}} {{ grad.sz_iw() }}
#define GRAD_SZ_W_{{ meta.optimizer_name | upper}} {{ grad.sz_w() }}
#define GRAD_SZ_RES_{{ meta.optimizer_name | upper}} {{ grad.sz_res() }}

/*
 * F1 sizes
 */
#define F1_SZ_ARG_{{ meta.optimizer_name | upper}} {{ f1.sz_arg() }}
#define F1_SZ_IW_{{ meta.optimizer_name | upper}} {{ f1.sz_iw() }}
#define F1_SZ_W_{{ meta.optimizer_name | upper}} {{ f1.sz_w() }}
#define F1_SZ_RES_{{ meta.optimizer_name | upper}} {{ f1.sz_res() }}

/*
 * F2 sizes
 */
#define F2_SZ_ARG_{{ meta.optimizer_name | upper}} {{ f2.sz_arg() }}
#define F2_SZ_IW_{{ meta.optimizer_name | upper}} {{ f2.sz_iw() }}
#define F2_SZ_W_{{ meta.optimizer_name | upper}} {{ f2.sz_w() }}
#define F2_SZ_RES_{{ meta.optimizer_name | upper}} {{ f2.sz_res() }}

/*
 * w_cost sizes
 */
#define W_COST_SZ_ARG_{{ meta.optimizer_name | upper}} {{ w_cost.sz_arg() }}
#define W_COST_SZ_IW_{{ meta.optimizer_name | upper}} {{ w_cost.sz_iw() }}
#define W_COST_SZ_W_{{ meta.optimizer_name | upper}} {{ w_cost.sz_w() }}
#define W_COST_SZ_RES_{{ meta.optimizer_name | upper}} {{ w_cost.sz_res() }}

/*
 * w1 sizes
 * Preconditioning of F1
 */
#define W1_SZ_ARG_{{ meta.optimizer_name | upper}} {{ w1.sz_arg() }}
#define W1_SZ_IW_{{ meta.optimizer_name | upper}} {{ w1.sz_iw() }}
#define W1_SZ_W_{{ meta.optimizer_name | upper}} {{ w1.sz_w() }}
#define W1_SZ_RES_{{ meta.optimizer_name | upper}} {{ w1.sz_res() }}

/*
 * w2 sizes
 * Preconditioning of F2
 */
#define W2_SZ_ARG_{{ meta.optimizer_name | upper}} {{ w2.sz_arg() }}
#define W2_SZ_IW_{{ meta.optimizer_name | upper}} {{ w2.sz_iw() }}
#define W2_SZ_W_{{ meta.optimizer_name | upper}} {{ w2.sz_w() }}
#define W2_SZ_RES_{{ meta.optimizer_name | upper}} {{ w2.sz_res() }}

/*
 * initial_penalty sizes
 */
#define INIT_PENALTY_SZ_ARG_{{ meta.optimizer_name | upper}} {{ init_penalty.sz_arg() }}
#define INIT_PENALTY_SZ_IW_{{ meta.optimizer_name | upper}} {{ init_penalty.sz_iw() }}
#define INIT_PENALTY_SZ_W_{{ meta.optimizer_name | upper}} {{ init_penalty.sz_w() }}
#define INIT_PENALTY_SZ_RES_{{ meta.optimizer_name | upper}} {{ init_penalty.sz_res() }}

