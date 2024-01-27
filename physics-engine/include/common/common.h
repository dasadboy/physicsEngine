#pragma once

#include <assert.h>
#include <concepts>
#include "math-defines.h"
#include <vector>
#include <iostream>
#include <unordered_set>
#include <iterator>

// from https://stackoverflow.com/questions/3692954/add-custom-messages-in-assert
#ifndef NDEBUG
#   define m_assert(Expr, Msg) \
    dbg::__M_Assert(#Expr, Expr, __FILE__, __LINE__, Msg)
#else
#   define m_assert(Expr, Msg) ;
#endif

namespace dbg
{
    void __M_Assert(const char* expr_str, bool expr, const char* file, int line, const char* msg);
}
