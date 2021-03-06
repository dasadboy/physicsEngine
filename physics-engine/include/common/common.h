#pragma once

#include <assert.h>
#include <concepts>
#include "math-defines.h"
#include <iostream>

// <--
#ifndef NDEBUG
#   define m_assert(Expr, Msg) \
    __M_Assert(#Expr, Expr, __FILE__, __LINE__, Msg)
#else
#   define m_assert(Expr, Msg) ;
#endif

void __M_Assert(const char* expr_str, bool expr, const char* file, int line, const char* msg)
{
    if (!expr)
    {
        std::cerr << "Assert failed:\t" << msg << "\n"
            << "Expected:\t" << expr_str << "\n"
            << "Source:\t\t" << file << ", line " << line << "\n";
        abort();
    }
}
// from https://stackoverflow.com/questions/3692954/add-custom-messages-in-assert -->
