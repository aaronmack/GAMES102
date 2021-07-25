// This file is generated by Ubpa::USRefl::AutoRefl

#pragma once

#include <USRefl/USRefl.h>

template<>
struct Ubpa::USRefl::TypeInfo<CanvasData> :
    TypeInfoBase<CanvasData>
{
#ifdef UBPA_USREFL_NOT_USE_NAMEOF
    static constexpr char name[11] = "CanvasData";
#endif
    static constexpr AttrList attrs = {};
    static constexpr FieldList fields = {
        Field {TSTR("points"), &Type::points},
        Field {TSTR("LagrangeResults"), &Type::LagrangeResults},
        Field {TSTR("GaussResults"), &Type::GaussResults},
        Field {TSTR("LeastSquaresResults"), &Type::LeastSquaresResults},
        Field {TSTR("RidgeRegressionResults"), &Type::RidgeRegressionResults},
        Field {TSTR("scrolling"), &Type::scrolling, AttrList {
            Attr {TSTR(UMeta::initializer), []()->Ubpa::valf2{ return { 0.f,0.f }; }},
        }},
        Field {TSTR("opt_enable_grid"), &Type::opt_enable_grid, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { true }; }},
        }},
        Field {TSTR("opt_enable_context_menu"), &Type::opt_enable_context_menu, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { true }; }},
        }},
        Field {TSTR("opt_lagrange"), &Type::opt_lagrange, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { true }; }},
        }},
        Field {TSTR("opt_gauss"), &Type::opt_gauss, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { false }; }},
        }},
        Field {TSTR("opt_least_squares"), &Type::opt_least_squares, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { false }; }},
        }},
        Field {TSTR("opt_ridge_regression"), &Type::opt_ridge_regression, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { false }; }},
        }},
        Field {TSTR("adding_line"), &Type::adding_line, AttrList {
            Attr {TSTR(UMeta::initializer), []()->bool{ return { false }; }},
        }},
        Field {TSTR("LeastSquaresM"), &Type::LeastSquaresM, AttrList {
            Attr {TSTR(UMeta::initializer), []()->int{ return 4; }},
        }},
        Field {TSTR("RidgeRegressionLamda"), &Type::RidgeRegressionLamda, AttrList {
            Attr {TSTR(UMeta::initializer), []()->float{ return 0.1; }},
        }},
    };
};

