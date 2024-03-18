/** ppTrans.cpp

   \copyright Copyright © CLEARSY 2023
   \license This file is part of ppTransSmt.

   ppTransSmt is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

    ppTransSmt is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ppTransSmt. If not, see <https://www.gnu.org/licenses/>.
*/
#include "ppTransTPTP.h"
#include "decomposition.h"
#include "predDesc.h"
#include "exprDesc.h"
#include "exprWriter.h"
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

ppTransException::ppTransException(const std::string &desc)
        : description(desc.data()) {};

ppTransException::~ppTransException() throw() {};

const char *ppTransException::what() const throw() { return description.c_str(); }

using namespace std;
namespace ppTransTPTP {

    class LocalEquations {
    public:
        void add_eq(Expr &&mp, Expr &&oexpr) {
            std::pair<Expr, Expr> pair = {std::move(mp), std::move(oexpr)};
            eqs.push_back(std::move(pair));
        }

        std::vector<TypedVar> vars;
        std::vector<std::pair<Expr, Expr> > eqs;
    };

    std::string localVarNameToString(const VarName &v) {
        switch (v.kind()) {
            case VarName::Kind::NoSuffix:
                return "L_" + v.prefix();
            case VarName::Kind::Tmp:
                return "X_" + std::to_string(-v.suffix());
            case VarName::Kind::WithSuffix:
                return "L_" + v.prefix() + "_" + std::to_string(v.suffix());
            case VarName::Kind::FreshId:
                assert(false);
        }
        assert(false); // unreachable
    }

    string globalVarNameToString(const VarName &v, int uid) {
        switch (v.kind()) {
            case VarName::Kind::NoSuffix:
                return "g_" + v.prefix() + "_" + std::to_string(uid);
            case VarName::Kind::WithSuffix:
                return "g_" + v.prefix() + "_" + std::to_string(v.suffix()) + "_" + std::to_string(uid);
            case VarName::Kind::Tmp:
                // Temporal variables cannot be global
                assert(false);
            case VarName::Kind::FreshId:
                assert(false);
        }
        assert(false); // unreachable
    }

    Expr getFreshVars(const BType &ty, std::vector<TypedVar> &vec) {
        if (ty.getKind() == BType::Kind::ProductType) {
            auto &pr = ty.toProductType();
            return Expr::makeBinaryExpr(
                    Expr::BinaryOp::Mapplet,
                    getFreshVars(pr.lhs, vec),
                    getFreshVars(pr.rhs, vec),
                    ty);
        } else if (ty.getKind() == BType::Kind::Struct) {
            auto &s = ty.toRecordType();
            std::vector<std::pair<std::string, Expr> > fields;
            for (auto &p: s.fields)
                fields.emplace_back(p.first, getFreshVars(p.second, vec));
            return Expr::makeRecord(std::move(fields), ty);
        } else {
            VarName v = VarName::makeTmp("X");
            vec.emplace_back(v, ty);
            return Expr::makeIdent(v, ty);
        }
    }

    std::string ppTrans(Context &ctx, const BType &ty, std::set<std::string> &used_ids);

    void
    ppTrans(std::ostringstream &str, Context &ctx, LocalEquations &eqs, const Expr &e, std::set<std::string> &used_ids);


    void toTypeList(std::ostringstream &str, Context &ctx, const BType &ty, std::set<std::string> &used_ids);

    void Context::pop_vars() {
        assert(!bv_stack.empty());
        bv_stack.pop_back();
    }

    void Context::push_vars(const std::vector<TypedVar> &vars) {
        bv_stack.push_back(vars);
    }

    // If the variable is not bound, then it is a global variable
    bool isBound(const std::vector<std::vector<TypedVar>> &stack, const VarName &v) {
        for (auto &vec: stack) {
            for (auto &tv: vec) {
                if (v == tv.name)
                    return true;
            }
        }
        return false;
    }

    void wrapStringInParenthesis(std::ostringstream &str) {
        std::string s = str.str();
        str.str("");
        str << "(" << s << ")";
    }

    void translateBinOp(std::ostringstream &str, std::string operatorText, Context &ctx, LocalEquations &eqs,
                        const Expr::BinaryExpr &e, std::set<std::string> &used_ids) {
        str << operatorText << "(";
        ppTrans(str, ctx, eqs, e.lhs, used_ids);
        str << ", ";
        ppTrans(str, ctx, eqs, e.rhs, used_ids);
        str << ")";
    }

    std::string Context::registerId(const VarName &v, const BType &ty, std::set<std::string> &used_ids) {
        if (v.kind() == VarName::Kind::Tmp)
            return localVarNameToString(v);

        if (!isBound(bv_stack, v)) {
            TypedVar tv = {v, ty};
            auto it = globalIdents.find(tv);
            std::string res;
            if (it == globalIdents.end()) {
                res = globalVarNameToString(v, static_cast<int>(globalIdents.size()));
                globalIdents.insert({tv, res});
                // Define a new variable of type name
                std::string type_name = ppTrans(*this, ty, used_ids);
                tptpDeclarations[res] = "tff(" + res + "_type, type, " + res + " : " + type_name + ").";
            } else {
                std::string not_used = ppTrans(*this, ty, used_ids); // necessary to update used_ids
                res = it->second;
            }
            used_ids.insert(res);
            return res;
        } else {
            return localVarNameToString(v); // local var
        }
    }

    // Function to Instantiate the membership operator for a given Powerset Type
    std::string Context::registerMem(const BType &ty, std::set<std::string> &used_ids) {
        assert(ty.getKind() == BType::Kind::PowerType);
        auto it = memberships.find(ty);
        std::string res;
        if (it != memberships.end()) {
            res = it->second;
        } else {
            res = "mem" + std::to_string(memberships.size());
            memberships[ty] = res;
            auto setType = setTypes.find(ty.toPowerType().content);
            if (setType == setTypes.end()){
                // The set type is not yet defined so we define it - before the membership operator
                // TPTP requires definitions to be before the use
                this->registerSetType(ty.toPowerType().content, used_ids);
                setType = setTypes.find(ty.toPowerType().content);
            }

            std::ostringstream str;
            str << "tff(" << res << "_type, type, " << res << " : (";
            toTypeList(str,*this,ty.toPowerType().content,used_ids);
            str << setType->second << ") > $o).";
            tptpDeclarations[res] = str.str();
        }
        used_ids.insert(res);
        return res;
    }

    // Function to Instantiate the membership operator for the Iteration Function
    std::string Context::registerIterate(const BType &ty, std::set<std::string> &used_ids) {
        auto it = iterates.find(ty);
        std::string res;
        if (it != iterates.end()) {
            res = it->second;
        } else {
            // The type of the iterate function is (P (P A) -> P A)
            res = "mem_it" + std::to_string(iterates.size());
            iterates[ty] = res;
            std::string sty = ppTrans(*this, ty, used_ids);

            assert(ty.getKind() == BType::Kind::PowerType);
            const BType::PowerType &pow = ty.toPowerType();
            assert(pow.content.getKind() == BType::Kind::ProductType);
            const BType::ProductType &prod = pow.content.toProductType();
            assert(prod.lhs == prod.rhs);

            std::vector<TypedVar> vars_x;
            Expr x = getFreshVars(prod.lhs, vars_x);
            std::vector<TypedVar> vars_y;
            Expr y = getFreshVars(prod.lhs, vars_y);
            std::vector<TypedVar> vars_z;
            Expr z = getFreshVars(prod.lhs, vars_z);

            std::string t_lst;
            std::string x_dec;
            std::string y_dec;
            std::string z_dec;
            std::string x_lst;
            std::string y_lst;
            std::string z_lst;
            std::string xy_eqs;
            // TODO - this is a bit ugly
            for (size_t i = 0; i < vars_x.size(); i++) {
                const TypedVar &x = vars_x[i];
                const TypedVar &y = vars_y[i];
                const TypedVar &z = vars_z[i];
                std::string ty = ppTrans(*this, x.type, used_ids);
                x_dec += " " + localVarNameToString(x.name) + " : " + ty;
                y_dec += " " + localVarNameToString(y.name) + " : " + ty;
                z_dec += " " + localVarNameToString(z.name) + " : " + ty;
                x_lst += " " + localVarNameToString(x.name);
                y_lst += " " + localVarNameToString(y.name);
                z_lst += " " + localVarNameToString(z.name);
                t_lst += " " + ty; // Type list for the iterate function
                xy_eqs += " (" + localVarNameToString(x.name) + " = " + localVarNameToString(y.name) + ")";
            }

            // Register the membership operator for the type of the iterate function
            std::string mem = this->registerMem(ty, used_ids);
            // Add the axiom for the iterate function
            std::ostringstream axiom1;
            axiom1 << "tff(" + res + "_type, type," + res + ": (" + t_lst + " * " + t_lst + " * " + sty +
                      " * $int) > $o)." << endl;
            axiom1 << "tff(" + res + "_axiom_1, axiom, ! [F : " + sty + ", X : " + t_lst + ", Y : " + t_lst + "] : "
                   << endl;
            axiom1 << "\t (" + res << "(X, Y, F,  0) => " + xy_eqs + " & ? [Z : " + t_lst + "] : (";
            axiom1 << mem << "(Z, Y, F))))." << endl;

            std::ostringstream axiom2;
            axiom2 << "tff(" + res + "_axiom_2, axiom, ! [F : " + sty + ", X : " << t_lst << "] : " << std::endl;
            axiom2 << "\t ? [ Y : " + t_lst + "] : (" << mem << "(X, Y, F) => " << res << "(X, Y, F,  0)))." << std::endl;

            std::ostringstream axiom3;
            axiom3 << "tff(" + res + "_axiom_3, axiom, ! [F : " + sty + ", N : $int, X :"
                   << t_lst << ", Y :" << t_lst  << "] : (" << endl;
            axiom3 << "\t $greatereq(N, 1) => (" << endl;
            axiom3 << "\t\t" << res << "(X, Y, F, N) => " << endl;
            axiom3 << "\t\t\t ? [Z : " << t_lst << "] : (" << res << "(X, Y, F, $difference(N, 1)) & " << mem << "(Z, Y, F)))))." << std::endl;

            std::ostringstream axiom4;
            axiom4 << "tff(" + res + "_axiom_4, axiom, ! [F : " + sty + ", N : $int, "
                  << "X : " << t_lst << ", Y :" << t_lst << "] : (" << std::endl;
            axiom4 << "\t $greatereq(N, 1) => (" << endl;
            axiom4 << "\t\t ?[Z : " << t_lst << "] : ((" << res << "(X, Y, F, $difference(N, 1)) & " << mem << "(Z, Y, F))" << endl;
            axiom4 << "\t\t\t => " << res << "(X, Y, F,  N)))))." << std::endl;

            tptpDeclarations[res] = axiom1.str() + axiom2.str() + axiom3.str() + axiom4.str();
        }
        used_ids.insert(res);
        return res;
    }

    std::string Context::registerRecordType(const BType &ty, std::set<std::string> &used_ids) {
        assert(ty.getKind() == BType::Kind::Struct);
        auto it = recordTypes.find(ty);
        std::string res;
        if (it != recordTypes.end()) {
            res = it->second;
        } else {
            // Records are represented as tuples
            res = "S" + std::to_string(recordTypes.size());
            recordTypes[ty] = res;
            tptpDeclarations[res] =
                    "tff(" + res + "_def, type, [" + std::to_string(ty.toRecordType().fields.size()) + "]).";
        }
        used_ids.insert(res);
        return res;
    }

    std::string Context::registerStringLiteral(const std::string &s, std::set<std::string> &used_ids) {
        auto it = stringLiterals.find(s);
        std::string res;
        if (it != stringLiterals.end()) {
            res = it->second;
        } else {
            res = "str" + std::to_string(stringLiterals.size());
            stringLiterals[s] = res;
            std::string tptp_type = "tff(" + res + "_type, type, " + res + ": String).";
            std::string tptp_axiom = "tff(" + res + "_def, axiom, " + res + " = " + s + ").";
            tptpDeclarations[res] = tptp_type + "\n" + tptp_axiom;
        }
        used_ids.insert(res);
        return res;
    }

    std::string Context::registerSetType(const BType &ty, std::set<std::string> &used_ids) {
        auto it = setTypes.find(ty);
        std::string res;
        if (it != setTypes.end()) {
            // Set is already declared
            res = it->second;
        } else {
            res = "set_" + std::to_string(setTypes.size());
            // Remember set
            setTypes[ty] = res;
            auto typeName = ppTrans(*this, ty, used_ids);
            auto tptp_type = "tff(" + res + "_type, type, " + res + " : $tType).";
            auto tptp_empty = "tff(" + res + "_empty, type, " + res + "_empty : " + res + ").";
            auto tptp_insert =
                    "tff(" + res + "_insert, type, " + res + " : (" + typeName + ") > " + res
                    + ").";
            tptpDeclarations[res] = tptp_type + "\n" + tptp_empty + "\n" + tptp_insert;
        }
        used_ids.insert(res);
        return res;
    }

    std::string Context::registerProductType(const BType &ty, std::set<std::string> &used_ids) {
        auto it = productTypes.find(ty);
        std::string res;
        if (it != productTypes.end()) {
            // Product is already declared
            res = it->second;
        } else {
            res = "prod_" + std::to_string(productTypes.size());
            // Remember set
            productTypes[ty] = res;
            auto pr = ty.toProductType();
            auto typeNameLhs = ppTrans(*this, pr.lhs, used_ids);
            auto typeNameRhs = ppTrans(*this, pr.rhs, used_ids);
            auto tptp_type = "tff(" + res + "_type, type, " + res + " : $tType).";
            auto tptp_empty = "tff(" + res + "_empty, type, " + res + "_empty : " + res + ").";
            auto tptp_insert =
                    "tff(" + res + "_insert, type, " + res + " : (" + typeNameLhs + " * " + typeNameRhs + ") > " + res
                    + ").";
            auto tptp_in =
                    "tff(" + res + "_in, type, " + res + "_in : (" + typeNameLhs + " * " + typeNameRhs + " * " + res +
                    ") > $o"
                    + ").";

            tptpDeclarations[res] = tptp_type + "\n" + tptp_empty + "\n" + tptp_insert + "\n" + tptp_in;
        }
        used_ids.insert(res);
        return res;
    }

    std::string
    Context::nameSimpleExpression(const Expr &e, LocalEquations &local_eqs, std::set<std::string> &used_ids) {
        assert(e.getType().getKind() != BType::Kind::ProductType);
        assert(e.getType().getKind() != BType::Kind::Struct);
        if (e.getTag() == Expr::EKind::Id) {
            return registerId(e.getId(), e.getType(), used_ids);
        } else {
            VarName v = VarName::makeTmp("x");
            local_eqs.vars.emplace_back(v, e.getType());
            local_eqs.add_eq(Expr::makeIdent(v, e.getType()), e.copy());
            return localVarNameToString(v);
        }
    }

    // Translate a BType to an TPTP type
    std::string ppTrans(Context &ctx, const BType &ty, std::set<std::string> &used_ids) {
        switch (ty.getKind()) {
            case BType::Kind::INTEGER:
                return "$int";
            case BType::Kind::BOOLEAN:
                // Booleans are of the type $o
                return "$o";
            case BType::Kind::REAL:
            case BType::Kind::FLOAT:
                // Float are considered reals
                return "$real";
            case BType::Kind::STRING:
                return "String";
            case BType::Kind::PowerType:
                // I need to keep track of this type
                return ctx.registerSetType(ty.toPowerType().content, used_ids);
            case BType::Kind::ProductType: {
                // I need to keep track of this type
                return ctx.registerProductType(ty, used_ids);
            }
            case BType::Kind::Struct: {
                //Todo - find out how to represent - the struct type
                const BType::RecordType &s = ty.toRecordType();
                std::string rec = ctx.registerRecordType(ty, used_ids);
                std::string accu = "(" + rec;
                for (size_t i = 0; i < s.fields.size(); ++i) {
                    auto &fd = s.fields.at(i);
                    accu += ppTrans(ctx, fd.second, used_ids);
                    if (i < s.fields.size() - 1)
                        accu += " * ";
                }
                return accu + ")";
            }
        }
        assert(false); // unreachable
    }

    std::vector<std::pair<std::string, Expr>> splitRecord(LocalEquations &local_eqs, const Expr &e) {
        assert(e.getType().getKind() == BType::Kind::Struct);
        if (e.getTag() == Expr::EKind::Record) {
            std::vector<std::pair<std::string, Expr>> res;
            for (auto &p: e.toRecordExpr().fields)
                res.push_back({p.first, p.second.copy()});
            return res;
        } else {
            assert(e.getTag() != Expr::EKind::Id);
            Expr rec = getFreshVars(e.getType(), local_eqs.vars);
            local_eqs.add_eq(rec.copy(), e.copy());
            assert(rec.getTag() == Expr::EKind::Record);
            return std::move(rec.toRecordExpr().fields);
        }
    }


    Expr getRecordField(LocalEquations &local_eqs, const Expr::RecordAccessExpr &e) {
        const auto &fields = splitRecord(local_eqs, e.rec);
        for (auto &p: fields) {
            if (p.first == e.label)
                return p.second.copy();
        }
        throw ppTransException("Error in ppTrans::getRecordField.");
    }

    void ppTrans(std::ostringstream &str, Context &ctx, LocalEquations &eqs, const Expr &e,
                 std::set<std::string> &used_ids) {
        assert(e.getType().getKind() != BType::Kind::ProductType);
        assert(e.getType().getKind() != BType::Kind::Struct);
        assert(e.getType().getKind() != BType::Kind::PowerType);

        switch (e.getTag()) {
            case Expr::EKind::INTEGER:
            case Expr::EKind::INT:
            case Expr::EKind::NAT:
            case Expr::EKind::NAT1:
            case Expr::EKind::NATURAL:
            case Expr::EKind::NATURAL1:
            case Expr::EKind::REAL:
            case Expr::EKind::FLOAT:
            case Expr::EKind::BOOL:
            case Expr::EKind::STRING:
            case Expr::EKind::EmptySet:
            case Expr::EKind::QuantifiedSet:
            case Expr::EKind::NaryExpr:
            case Expr::EKind::Struct:
            case Expr::EKind::Record:
            case Expr::EKind::Record_Field_Update:
            case Expr::EKind::Successor:
            case Expr::EKind::Predecessor:
                throw ppTransException("ppTrans: type error.");
            case Expr::EKind::TRUE:
                str << "$true";
                return;
            case Expr::EKind::FALSE:
                str << "$false";
                return;
            case Expr::EKind::BooleanExpr:
                ppTrans(str, ctx, e.toBooleanExpr(), used_ids);
                return;
            case Expr::EKind::StringLiteral:
                str << ctx.registerStringLiteral(e.getStringLiteral(), used_ids);
                return;
            case Expr::EKind::RealLiteral: {
                auto &d = e.getRealLiteral();
                str << d.integerPart << "." << d.fractionalPart;
                return;
            }
            case Expr::EKind::Record_Field_Access:
                ppTrans(str, ctx, eqs, getRecordField(eqs, e.toRecordAccess()), used_ids);
                return;
            case Expr::EKind::MinInt:
                str << "min_int";
                return;
            case Expr::EKind::MaxInt:
                str << "max_int";
                return;
            case Expr::EKind::Id:
                str << ctx.registerId(e.getId(), e.getType(), used_ids);
                return;
            case Expr::EKind::IntegerLiteral: {
                auto &lit = e.getIntegerLiteral();
                if (lit.at(0) == '-')
                    str << "$uminus(" << lit.substr(1, std::string::npos) << ")";
                else
                    str << lit;
                return;
            }
            case Expr::EKind::BinaryExpr: {
                auto &b = e.toBinaryExpr();
                switch (b.op) {
                    // Arithmetic constructs
                    case Expr::BinaryOp::IMultiplication:
                    case Expr::BinaryOp::RMultiplication:
                        translateBinOp(str, "$product", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::IExponentiation:
                        translateBinOp(str, "exp", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::IAddition:
                    case Expr::BinaryOp::RAddition:
                        translateBinOp(str, "$sum", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::ISubtraction:
                    case Expr::BinaryOp::RSubtraction:
                        translateBinOp(str, "$difference", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::IDivision:
                        translateBinOp(str, "divB", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::Modulo:
                        //TODO - not sure
                        translateBinOp(str, "$remainder_f", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::RExponentiation:
                        translateBinOp(str, "rexp", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::RDivision:
                        // FIXME: This is a hack to make the translation work
                        translateBinOp(str, "$qoutient_f", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::FDivision:
                        // FIXME: This is a hack to make the translation work
                        // Float operators are made to reals
                        translateBinOp(str, "$qoutient_f", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::FSubtraction:
                        // Float operators are made to reals
                        translateBinOp(str, "$difference", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::FAddition:
                        // Float operators are made to reals
                        translateBinOp(str, "$sum", ctx, eqs, b, used_ids);
                        return;
                    case Expr::BinaryOp::FMultiplication:
                        // Float operators are made to reals
                        translateBinOp(str, "$product", ctx, eqs, b, used_ids);
                        return;
                        // Other constructs
                    case Expr::BinaryOp::Application: {
                        str << ctx.nameSimpleExpression(e, eqs, used_ids);
                        return;
                    }
                        // Errors
                    case Expr::BinaryOp::Cartesian_Product:
                    case Expr::BinaryOp::Set_Difference:
                    case Expr::BinaryOp::Mapplet:
                    case Expr::BinaryOp::Partial_Functions:
                    case Expr::BinaryOp::Partial_Surjections:
                    case Expr::BinaryOp::Total_Functions:
                    case Expr::BinaryOp::Total_Surjections:
                    case Expr::BinaryOp::Head_Insertion:
                    case Expr::BinaryOp::Interval:
                    case Expr::BinaryOp::Intersection:
                    case Expr::BinaryOp::Head_Restriction:
                    case Expr::BinaryOp::Composition:
                    case Expr::BinaryOp::Surcharge:
                    case Expr::BinaryOp::Relations:
                    case Expr::BinaryOp::Tail_Insertion:
                    case Expr::BinaryOp::Domain_Subtraction:
                    case Expr::BinaryOp::Domain_Restriction:
                    case Expr::BinaryOp::Partial_Injections:
                    case Expr::BinaryOp::Total_Injections:
                    case Expr::BinaryOp::Partial_Bijections:
                    case Expr::BinaryOp::Total_Bijections:
                    case Expr::BinaryOp::Direct_Product:
                    case Expr::BinaryOp::Parallel_Product:
                    case Expr::BinaryOp::Union:
                    case Expr::BinaryOp::Tail_Restriction:
                    case Expr::BinaryOp::Concatenation:
                    case Expr::BinaryOp::Range_Restriction:
                    case Expr::BinaryOp::Range_Subtraction:
                    case Expr::BinaryOp::Image:
                    case Expr::BinaryOp::First_Projection:
                    case Expr::BinaryOp::Second_Projection:
                    case Expr::BinaryOp::Iteration:
                        throw ppTransException("ppTrans: type error.");
                    case Expr::BinaryOp::Const:
                    case Expr::BinaryOp::Rank:
                    case Expr::BinaryOp::Father:
                    case Expr::BinaryOp::Subtree:
                    case Expr::BinaryOp::Arity:
                        throw ppTransException("ppTrans: tree constructs not implemented.");
                }
            }
            case Expr::EKind::UnaryExpr: {
                auto &u = e.toUnaryExpr();
                switch (u.op) {
                    // Arithmetic constructs
                    case Expr::UnaryOp::IMinus:
                    case Expr::UnaryOp::RMinus:
                        str << "$uminus(";
                        ppTrans(str, ctx, eqs, u.content, used_ids);
                        str << ")";
                        return;
                    case Expr::UnaryOp::Real:
                        str << "$to_real(";
                        ppTrans(str, ctx, eqs, u.content, used_ids);
                        str << ")";
                        return;
                    case Expr::UnaryOp::Floor:
                        str << "$floor(";
                        ppTrans(str, ctx, eqs, u.content, used_ids);
                        str << ")";
                        return;
                    case Expr::UnaryOp::Ceiling:
                        str << "$ceiling(";
                        ppTrans(str, ctx, eqs, u.content, used_ids);
                        str << ")";
                        return;
                        // Other constructs
                    case Expr::UnaryOp::IMaximum:
                    case Expr::UnaryOp::IMinimum:
                    case Expr::UnaryOp::Cardinality:
                    case Expr::UnaryOp::Size:
                    case Expr::UnaryOp::First:
                    case Expr::UnaryOp::Last:
                    case Expr::UnaryOp::RMaximum:
                    case Expr::UnaryOp::RMinimum: {
                        str << ctx.nameSimpleExpression(e, eqs, used_ids);
                        return;
                    }
                        // Errors
                    case Expr::UnaryOp::Domain:
                    case Expr::UnaryOp::Range:
                    case Expr::UnaryOp::Subsets:
                    case Expr::UnaryOp::Non_Empty_Subsets:
                    case Expr::UnaryOp::Finite_Subsets:
                    case Expr::UnaryOp::Non_Empty_Finite_Subsets:
                    case Expr::UnaryOp::Union:
                    case Expr::UnaryOp::Intersection:
                    case Expr::UnaryOp::Sequences:
                    case Expr::UnaryOp::Non_Empty_Sequences:
                    case Expr::UnaryOp::Injective_Sequences:
                    case Expr::UnaryOp::Non_Empty_Injective_Sequences:
                    case Expr::UnaryOp::Inverse:
                    case Expr::UnaryOp::Permutations:
                    case Expr::UnaryOp::Identity:
                    case Expr::UnaryOp::Closure:
                    case Expr::UnaryOp::Transitive_Closure:
                    case Expr::UnaryOp::Tail:
                    case Expr::UnaryOp::Front:
                    case Expr::UnaryOp::Reverse:
                    case Expr::UnaryOp::Concatenation:
                    case Expr::UnaryOp::Rel:
                    case Expr::UnaryOp::Fnc:
                        throw ppTransException("ppTrans: type error.");
                    case Expr::UnaryOp::Tree:
                    case Expr::UnaryOp::Btree:
                    case Expr::UnaryOp::Top:
                    case Expr::UnaryOp::Sons:
                    case Expr::UnaryOp::Prefix:
                    case Expr::UnaryOp::Postfix:
                    case Expr::UnaryOp::Sizet:
                    case Expr::UnaryOp::Mirror:
                    case Expr::UnaryOp::Left:
                    case Expr::UnaryOp::Right:
                    case Expr::UnaryOp::Infix:
                    case Expr::UnaryOp::Bin:
                        throw ppTransException("ppTrans_a: tree constructs not implemented.");
                }
                assert(false); // unreachable
            }
            case Expr::EKind::TernaryExpr:
                throw ppTransException("ppTrans: tree constructs not implemented.");
            case Expr::EKind::QuantifiedExpr: {
                auto &q = e.toQuantiedExpr();
                switch (q.op) {
                    case Expr::QuantifiedOp::Lambda:
                    case Expr::QuantifiedOp::Intersection:
                    case Expr::QuantifiedOp::Union:
                        throw ppTransException("ppTrans: type error.");
                    case Expr::QuantifiedOp::RSum: {
                        assert(!q.vars.empty());
                        BType tdom = q.vars[0].type;
                        for (size_t i = 1; i < q.vars.size(); i++)
                            tdom = BType::PROD(tdom, q.vars[i].type);
                        Expr l = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::Lambda, q.vars, q.cond.copy(),
                                                          q.body.copy(),
                                                          BType::POW(BType::PROD(tdom, q.body.getType())));
                        Expr ran = Expr::makeUnaryExpr(Expr::UnaryOp::Range, std::move(l), BType::POW_REAL);
                        str << "rsum(" << ctx.nameSimpleExpression(ran, eqs, used_ids) << ")";
                        return;
                    }
                    case Expr::QuantifiedOp::RProduct: {
                        assert(!q.vars.empty());
                        BType tdom = q.vars[0].type;
                        for (size_t i = 1; i < q.vars.size(); i++)
                            tdom = BType::PROD(tdom, q.vars[i].type);
                        Expr l = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::Lambda, q.vars, q.cond.copy(),
                                                          q.body.copy(),
                                                          BType::POW(BType::PROD(tdom, q.body.getType())));
                        Expr ran = Expr::makeUnaryExpr(Expr::UnaryOp::Range, std::move(l), BType::POW_REAL);
                        str << "rprod(" << ctx.nameSimpleExpression(ran, eqs, used_ids) << ")";
                        return;
                    }
                    case Expr::QuantifiedOp::ISum: {
                        assert(!q.vars.empty());
                        BType tdom = q.vars[0].type;
                        for (size_t i = 1; i < q.vars.size(); i++)
                            tdom = BType::PROD(tdom, q.vars[i].type);
                        Expr l = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::Lambda, q.vars, q.cond.copy(),
                                                          q.body.copy(),
                                                          BType::POW(BType::PROD(tdom, q.body.getType())));
                        Expr ran = Expr::makeUnaryExpr(Expr::UnaryOp::Range, std::move(l), BType::POW_INT);
                        str << "isum(" << ctx.nameSimpleExpression(ran, eqs, used_ids) << ")";
                        return;
                    }
                    case Expr::QuantifiedOp::IProduct: {
                        assert(!q.vars.empty());
                        BType tdom = q.vars[0].type;
                        for (size_t i = 1; i < q.vars.size(); i++)
                            tdom = BType::PROD(tdom, q.vars[i].type);
                        Expr l = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::Lambda, q.vars, q.cond.copy(),
                                                          q.body.copy(),
                                                          BType::POW(BType::PROD(tdom, q.body.getType())));
                        Expr ran = Expr::makeUnaryExpr(Expr::UnaryOp::Range, std::move(l), BType::POW_INT);
                        str << "iprod(" << ctx.nameSimpleExpression(ran, eqs, used_ids) << ")";
                        return;
                    }
                }
            }
        }
        assert(false); // unreachable
    }

    // Split an expression with a product type into two subcomponents.
    std::pair<Expr, Expr> splitPair(LocalEquations &local_eqs, const Expr &e) {
        assert(e.getType().getKind() == BType::Kind::ProductType);
        if (e.getTag() == Expr::EKind::BinaryExpr) {
            auto &b = e.toBinaryExpr();
            if (b.op == Expr::BinaryOp::Mapplet) {
                return {b.lhs.copy(), b.rhs.copy()};
            }
        }
//        assert(e.getTag() != Expr::EKind::Id);
        assert(e.getTag() != Expr::EKind::Struct);
        Expr mp = getFreshVars(e.getType(), local_eqs.vars);
        local_eqs.add_eq(mp.copy(), e.copy());
        assert(mp.getTag() == Expr::EKind::BinaryExpr);
        auto &b = mp.toBinaryExpr();
        assert(b.op == Expr::BinaryOp::Mapplet);
        return {std::move(b.lhs), std::move(b.rhs)};
    }

    void
    toIdentList(std::ostream &str, Context &ctx, LocalEquations &eqs, const Expr &e, std::set<std::string> &used_ids) {
        switch (e.getType().getKind()) {
            case BType::Kind::ProductType: {
                auto pair = splitPair(eqs, e);
                toIdentList(str, ctx, eqs, pair.first, used_ids);
                toIdentList(str, ctx, eqs, pair.second, used_ids);
                return;
            }
            case BType::Kind::Struct: {
                auto fields = splitRecord(eqs, e);
                for (auto &p: fields)
                    toIdentList(str, ctx, eqs, p.second, used_ids);
                return;
            }
            default: {
                // The arguments of a function is a comma seperated string
                str << ctx.nameSimpleExpression(e, eqs, used_ids) << ", ";
            }
        }
    }

    void toTypeList(std::ostringstream &str, Context &ctx, const BType &ty, std::set<std::string> &used_ids) {
        switch (ty.getKind()) {
            case BType::Kind::INTEGER:
            case BType::Kind::STRING:
            case BType::Kind::BOOLEAN:
            case BType::Kind::REAL:
            case BType::Kind::FLOAT:
            case BType::Kind::PowerType:
                str << ppTrans(ctx, ty, used_ids) << " * ";
                return;
            case BType::Kind::ProductType: {
                auto &pr = ty.toProductType();
                toTypeList(str, ctx, pr.lhs, used_ids);
                toTypeList(str, ctx, pr.rhs, used_ids);
                return;
            }
            case BType::Kind::Struct: {
                for (auto &fd: ty.toRecordType().fields)
                    toTypeList(str, ctx, fd.second, used_ids);
                return;
            }
        }
        assert(false); // unreachable
    }

    void ppTrans_mem(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr &rhs,
                     std::set<std::string> &used_ids);

    void
    add_local_defs(std::ostringstream &str, Context &env, const LocalEquations &local_eqs, const std::ostringstream &f,
                   std::set<std::string> &used_ids);

    // Convert an equality [lhs]=[rhs] into a TPTP string
    void ppTrans_eq(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr &rhs,
                    std::set<std::string> &used_ids) {
        //assert(BType::weak_eq(lhs.getType(),rhs.getType()));
        const BType &ty = lhs.getType();

        // f(a) = b ----> (a,b) : f
        if (lhs.getTag() == Expr::EKind::BinaryExpr) {
            auto &b = lhs.toBinaryExpr();
            if (b.op == Expr::BinaryOp::Application) {
                BType ty_mp = BType::PROD(
                        b.rhs.getType(),
                        rhs.getType());
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, b.rhs.copy(), rhs.copy(), ty_mp);
                return ppTrans_mem(str, ctx, mp, b.lhs, used_ids);
            }
        }
        // b = f(a) ----> f(a) = b
        if (rhs.getTag() == Expr::EKind::BinaryExpr) {
            auto &b = rhs.toBinaryExpr();
            if (b.op == Expr::BinaryOp::Application) {
                return ppTrans_eq(str, ctx, rhs, lhs, used_ids);
            }
        }
        if (lhs.getTag() == Expr::EKind::UnaryExpr) {
            auto &u = lhs.toUnaryExpr();
            switch (u.op) {
                case Expr::UnaryOp::Last: {
                    // last(a) = b ----> a(size(a)) = b
                    Expr app = Expr::makeBinaryExpr(
                            Expr::BinaryOp::Application,
                            u.content.copy(),
                            Expr::makeUnaryExpr(Expr::UnaryOp::Size, u.content.copy(), BType::INT), // FIXME
                            rhs.getType());
                    return ppTrans_eq(str, ctx, app, rhs, used_ids);
                }
                case Expr::UnaryOp::First: {
                    // first(a) = b ---> a(1) = b
                    Expr app = Expr::makeBinaryExpr(
                            Expr::BinaryOp::Application,
                            u.content.copy(),
                            Expr::makeInteger("1"),
                            rhs.getType());
                    return ppTrans_eq(str, ctx, app, rhs, used_ids);
                }
                case Expr::UnaryOp::IMinimum: {
                    // min(a) = b ---> b:a & forall x:a. b<=x
                    // with lhs = min(a), u.content = a, rhs = b
                    std::vector<TypedVar> vars;
                    Expr id_x = getFreshVars(BType::INT, vars);
                    std::vector<Pred> vec;
                    vec.push_back(
                            Pred::makeExprComparison(Pred::ComparisonOp::Membership, rhs.copy(), u.content.copy()));
                    vec.push_back(Pred::makeForall(vars,
                                                   Pred::makeImplication(
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Membership,
                                                                                    id_x.copy(),
                                                                                    u.content.copy()), // FIXME
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Ile, rhs.copy(),
                                                                                    id_x.copy()))));
                    Pred p = Pred::makeConjunction(std::move(vec));
                    return ppTrans(str, ctx, p, used_ids);
                }
                case Expr::UnaryOp::IMaximum: {
                    // max(a) = b ---> b:a & forall x:a.b>=x
                    // with lhs = min(a), u.content = a, rhs = b
                    std::vector<TypedVar> vars;
                    Expr id_x = getFreshVars(BType::INT, vars);
                    std::vector<Pred> vec;
                    vec.push_back(
                            Pred::makeExprComparison(Pred::ComparisonOp::Membership, rhs.copy(), u.content.copy()));
                    vec.push_back(Pred::makeForall(vars,
                                                   Pred::makeImplication(
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Membership,
                                                                                    id_x.copy(),
                                                                                    u.content.copy()), // FIXME
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Ige, rhs.copy(),
                                                                                    id_x.copy()))));
                    Pred p = Pred::makeConjunction(std::move(vec));
                    return ppTrans(str, ctx, p, used_ids);
                }
                case Expr::UnaryOp::RMinimum: {
                    // min(a) = b ---> b:a & forall x:a. b<=x
                    // with lhs = min(a), u.content = a, rhs = b
                    std::vector<TypedVar> vars;
                    Expr id_x = getFreshVars(BType::REAL, vars);
                    std::vector<Pred> vec;
                    vec.push_back(
                            Pred::makeExprComparison(Pred::ComparisonOp::Membership, rhs.copy(), u.content.copy()));
                    vec.push_back(Pred::makeForall(vars,
                                                   Pred::makeImplication(
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Membership,
                                                                                    id_x.copy(),
                                                                                    u.content.copy()), // FIXME
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Rle, rhs.copy(),
                                                                                    id_x.copy()))));
                    Pred p = Pred::makeConjunction(std::move(vec));
                    return ppTrans(str, ctx, p, used_ids);
                }
                case Expr::UnaryOp::RMaximum: {
                    // max(a) = b ---> b:a & forall x:a.b>=x
                    // with lhs = min(a), u.content = a, rhs = b
                    std::vector<TypedVar> vars;
                    Expr id_x = getFreshVars(BType::REAL, vars);
                    std::vector<Pred> vec;
                    vec.push_back(
                            Pred::makeExprComparison(Pred::ComparisonOp::Membership, rhs.copy(), u.content.copy()));
                    vec.push_back(Pred::makeForall(vars,
                                                   Pred::makeImplication(
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Membership,
                                                                                    id_x.copy(),
                                                                                    u.content.copy()), // FIXME
                                                           Pred::makeExprComparison(Pred::ComparisonOp::Rge, rhs.copy(),
                                                                                    id_x.copy()))));
                    Pred p = Pred::makeConjunction(std::move(vec));
                    return ppTrans(str, ctx, p, used_ids);
                }
                case Expr::UnaryOp::Size: {
                    // size(a) = b ---> dom(a) = 1..b
                    Expr itvl = Expr::makeBinaryExpr(
                            Expr::BinaryOp::Interval,
                            Expr::makeInteger("1"),
                            rhs.copy(),
                            BType::POW(BType::INT));
                    Expr dom = Expr::makeUnaryExpr(
                            Expr::UnaryOp::Domain,
                            u.content.copy(),
                            BType::POW(BType::INT));
                    return ppTrans_eq(str, ctx, dom, itvl, used_ids);
                }
                case Expr::UnaryOp::Cardinality: {
                    // card(a) = b ---> exists f. f:(a >>-> 1..b)
                    Expr itvl = Expr::makeBinaryExpr(
                            Expr::BinaryOp::Interval,
                            Expr::makeInteger("1"),
                            rhs.copy(),
                            BType::POW(BType::INT));
                    BType ty_a = u.content.getType();
                    assert(ty_a.getKind() == BType::Kind::PowerType);
                    BType ty_f = BType::POW(BType::PROD(ty_a.toPowerType().content, BType::INT));
                    std::vector<TypedVar> vars;
                    Expr id_f = getFreshVars(ty_f, vars);
                    Pred p = Pred::makeExists(vars,
                                              Pred::makeExprComparison(
                                                      Pred::ComparisonOp::Membership,
                                                      std::move(id_f),
                                                      Expr::makeBinaryExpr(
                                                              Expr::BinaryOp::Total_Bijections,
                                                              u.content.copy(),
                                                              std::move(itvl),
                                                              BType::POW(ty_f)
                                                      )));
                    return ppTrans(str, ctx, p, used_ids);
                }
                default:
                    break;
            }
        }
        if (rhs.getTag() == Expr::EKind::UnaryExpr) {
            auto &u = rhs.toUnaryExpr();
            switch (u.op) {
                case Expr::UnaryOp::Last:
                case Expr::UnaryOp::First:
                case Expr::UnaryOp::IMinimum:
                case Expr::UnaryOp::IMaximum:
                case Expr::UnaryOp::RMinimum:
                case Expr::UnaryOp::RMaximum:
                case Expr::UnaryOp::Size:
                case Expr::UnaryOp::Cardinality:
                    return ppTrans_eq(str, ctx, rhs, lhs, used_ids);
                default:
                    break;
            }
        }

        switch (ty.getKind()) {
            // All equality constraints are translated to the following form:
            case BType::Kind::INTEGER:
            case BType::Kind::STRING:
            case BType::Kind::FLOAT:
            case BType::Kind::REAL: {
                LocalEquations local_eqs;
                std::ostringstream str2;
                ppTrans(str2, ctx, local_eqs, lhs, used_ids);
                str2 << " = ";
                ppTrans(str2, ctx, local_eqs, rhs, used_ids);
                wrapStringInParenthesis(str2);
                return add_local_defs(str, ctx, local_eqs, str2, used_ids);
            }
            case BType::Kind::BOOLEAN: {
                LocalEquations local_eqs;
                std::ostringstream str2;
                ppTrans(str2, ctx, local_eqs, lhs, used_ids);
                wrapStringInParenthesis(str2);
                str2 << " <=> ";
                ppTrans(str2, ctx, local_eqs, rhs, used_ids);
                wrapStringInParenthesis(str2);
                wrapStringInParenthesis(str2);
                return add_local_defs(str, ctx, local_eqs, str2, used_ids);
            }
            case BType::Kind::PowerType: {
                BType ty_x = ty.toPowerType().content;
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(ty_x, vars);
                Pred p = Pred::makeForall(vars, Pred::makeEquivalence(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, x.copy(), lhs.copy()),
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, x.copy(), rhs.copy())));
                return ppTrans(str, ctx, p, used_ids);
            }
            case BType::Kind::ProductType: {
                LocalEquations eqs;
                std::ostringstream str2;
                std::pair<Expr, Expr> plhs = splitPair(eqs, lhs);
                std::pair<Expr, Expr> prhs = splitPair(eqs, rhs);
                //assert(plhs.first.getType() == prhs.first.getType());
                //assert(plhs.second.getType() == prhs.second.getType());
                ppTrans_eq(str2, ctx, plhs.first, prhs.first, used_ids);
                str2 << " & ";
                ppTrans_eq(str2, ctx, plhs.second, prhs.second, used_ids);
                wrapStringInParenthesis(str2);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case BType::Kind::Struct: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto fds1 = splitRecord(eqs, lhs);
                auto fds2 = splitRecord(eqs, rhs);
                std::vector<Pred> conj;
                assert(fds1.size() == fds2.size());
                for (size_t i = 0; i < fds1.size(); i++) {
                    assert(fds1[i].first == fds2[i].first);
                    conj.push_back(
                            Pred::makeExprComparison(
                                    Pred::ComparisonOp::Equality,
                                    fds1[i].second.copy(),
                                    fds2[i].second.copy()));
                }
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(conj)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
        }
        assert(false); // unreachable
    }

    Pred isfunc(const Expr &f) {
        const BType &ty_f = f.getType();
        assert(ty_f.getKind() == BType::Kind::PowerType);
        BType ty_mp = ty_f.toPowerType().content;
        assert(ty_mp.getKind() == BType::Kind::ProductType);
        auto &pr = ty_mp.toProductType();
        std::vector<TypedVar> vars;
        Expr a = getFreshVars(pr.lhs, vars);
        Expr b = getFreshVars(pr.rhs, vars);
        Expr c = getFreshVars(pr.rhs, vars);
        std::vector<Pred> vec;
        vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership,
                                               Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, a.copy(), b.copy(), ty_mp),
                                               f.copy()));
        vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership,
                                               Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, a.copy(), c.copy(), ty_mp),
                                               f.copy())); // FIXME
        return Pred::makeForall(vars,
                                Pred::makeImplication(
                                        Pred::makeConjunction(std::move(vec)),
                                        Pred::makeExprComparison(Pred::ComparisonOp::Equality, b.copy(), c.copy())
                                )
        );
    }

    Pred isSurj(const Expr &f, const Expr &ran) {
        return Pred::makeExprComparison(Pred::ComparisonOp::Subset, ran.copy(),
                                        Expr::makeUnaryExpr(Expr::UnaryOp::Range, f.copy(), ran.getType()));
    }

    Pred isInj(const Expr &f) {
        const BType &ty_f = f.getType();
        assert(ty_f.getKind() == BType::Kind::PowerType);
        BType ty_mp = ty_f.toPowerType().content;
        assert(ty_mp.getKind() == BType::Kind::ProductType);
        auto &pr = ty_mp.toProductType();
        return isfunc(Expr::makeUnaryExpr(Expr::UnaryOp::Inverse, f.copy(), BType::POW(BType::PROD(pr.rhs, pr.lhs))));
    }

    // A function to instantiate the membership predicate for a given type
    void makeMem(std::ostringstream &str, Context &ctx, LocalEquations &eqs, const Expr &lhs, const std::string &set,
                 const BType &set_type, std::set<std::string> &used_ids) {
        //assert(set_type.getKind() == BType::Kind::PowerType);
        str << ctx.registerMem(set_type, used_ids) << "(";
        toIdentList(str, ctx, eqs, lhs, used_ids);
        str << set << ")";
    }

    void ppTrans_mem(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr::BinaryExpr &set,
                     std::set<std::string> &used_ids) {
        BType ty_rhs = BType::POW(lhs.getType());
        switch (set.op) {
            case Expr::BinaryOp::Cartesian_Product: {
                LocalEquations eqs;
                std::ostringstream str2;
                std::pair<Expr, Expr> pair = splitPair(eqs, lhs);
                str2 << "(";
                ppTrans_mem(str2, ctx, pair.first, set.lhs, used_ids);
                str2 << " & "; // Updated
                ppTrans_mem(str2, ctx, pair.second, set.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Partial_Functions: {
                BType ty_lhs = lhs.getType();
                str << "(";
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Relations, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & "; // Updated
                ppTrans(str, ctx, isfunc(lhs), used_ids);
                str << ")";
                return;
            }
            case Expr::BinaryOp::Partial_Surjections: {
                BType ty_lhs = lhs.getType();
                str << "( ";
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Partial_Functions, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & ";
                ppTrans(str, ctx, isSurj(lhs, set.rhs), used_ids); // FIXME
                str << ")";
                return;
            }
            case Expr::BinaryOp::Partial_Injections: {
                BType ty_lhs = lhs.getType();
                str << "(";
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Partial_Functions, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & ";
                ppTrans(str, ctx, isInj(lhs), used_ids);
                str << ")";
                return;
            }
            case Expr::BinaryOp::Total_Injections: {
                BType ty_lhs = lhs.getType();
                str << "(";
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Total_Functions, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & ";
                ppTrans(str, ctx, isInj(lhs), used_ids);
                str << ")";
                return;
            }
            case Expr::BinaryOp::Partial_Bijections: {
                BType ty_lhs = lhs.getType();
                str << "(";
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Partial_Surjections, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & ";
                ppTrans(str, ctx, isInj(lhs), used_ids);
                str << ")";
                return;
            }
            case Expr::BinaryOp::Total_Bijections: {
                BType ty_lhs = lhs.getType();
                str << "(";
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Total_Surjections, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & ";
                ppTrans(str, ctx, isInj(lhs), used_ids);
                str << ")";
                return;
            }
            case Expr::BinaryOp::Set_Difference: {
                str << "(";
                ppTrans_mem(str, ctx, lhs, set.lhs, used_ids);
                str << " & (~(";
                ppTrans_mem(str, ctx, lhs, set.rhs, used_ids);
                str << ")))";
                return;
            }
            case Expr::BinaryOp::Total_Functions: {
                // f: A-->B  ==>  exists x. (x=f & x:isFunc(x) & A = dom(x) & ran(x) <: B)
                const BType &ty_lhs = lhs.getType();
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(ty_lhs, vars);
                ctx.push_vars(vars);
                str << "(? [";
                for (size_t i = 0; i < vars.size(); ++i) {
                    const auto &v = vars.at(i);
                    str << localVarNameToString(v.name) << " : " << ppTrans(ctx, v.type, used_ids);
                    if (i < vars.size() - 1)
                        str << ", ";
                }
                str << "] : (";
                ppTrans_eq(str, ctx, x, lhs, used_ids);
                str << " & ";
                ppTrans(str, ctx, isfunc(x), used_ids);
                str << " & ";
                ppTrans(str, ctx, Pred::makeExprComparison(Pred::ComparisonOp::Equality, set.lhs.copy(),
                                                           Expr::makeUnaryExpr(Expr::UnaryOp::Domain, x.copy(),
                                                                               set.lhs.getType())), used_ids);
                str << " & ";
                ppTrans(str, ctx, Pred::makeExprComparison(Pred::ComparisonOp::Subset,
                                                           Expr::makeUnaryExpr(Expr::UnaryOp::Range, x.copy(),
                                                                               set.rhs.getType()), set.rhs.copy()),
                        used_ids);
                str << "))";
                ctx.pop_vars();
                return;
            }
            case Expr::BinaryOp::Total_Surjections: {
                BType ty_lhs = lhs.getType();
                ppTrans_mem(str, ctx, lhs,
                            Expr::makeBinaryExpr(Expr::BinaryOp::Total_Functions, set.lhs.copy(), set.rhs.copy(),
                                                 BType::POW(ty_lhs)), used_ids);
                str << " & ";
                ppTrans(str, ctx, isSurj(lhs, set.rhs), used_ids);
                return;
            }
            case Expr::BinaryOp::Head_Insertion: {
                std::vector<Expr> vec;
                vec.push_back(set.lhs.copy());
                Expr sing = Expr::makeNaryExpr(Expr::NaryOp::Sequence, std::move(vec), ty_rhs);
                Expr new_rhs = Expr::makeBinaryExpr(Expr::BinaryOp::Concatenation, std::move(sing), set.rhs.copy(),
                                                    ty_rhs);
                return ppTrans_mem(str, ctx, lhs, new_rhs, used_ids);
            }
            case Expr::BinaryOp::Interval: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greatereq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, set.lhs, used_ids);
                str2 << ") & $lesseq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids); // FIXME
                str2 << ", ";
                ppTrans(str2, ctx, eqs, set.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Intersection: {
                ppTrans_mem(str, ctx, lhs, set.lhs, used_ids);
                str << " & "; // FIXME
                ppTrans_mem(str, ctx, lhs, set.rhs, used_ids);
                return;
            }
            case Expr::BinaryOp::Head_Restriction: {
                Expr itbl = Expr::makeBinaryExpr(Expr::BinaryOp::Interval, Expr::makeInteger("1"), set.rhs.copy(),
                                                 BType::POW(BType::INT));
                Expr new_rhs = Expr::makeBinaryExpr(Expr::BinaryOp::Domain_Restriction, std::move(itbl), set.lhs.copy(),
                                                    ty_rhs);
                return ppTrans_mem(str, ctx, lhs, new_rhs, used_ids);
            }
            case Expr::BinaryOp::Composition: {
                BType ty_b_lhs = set.lhs.getType();
                assert(ty_b_lhs.getKind() == BType::Kind::PowerType);
                BType prod = ty_b_lhs.toPowerType().content;
                assert(prod.getKind() == BType::Kind::ProductType);
                auto &pr = prod.toProductType();
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(pr.rhs, vars);
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                Expr mp_1 = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(pair.first), x.copy(),
                                                 BType::PROD(pair.first.getType(), pr.rhs));
                Expr mp_2 = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(x), std::move(pair.second),
                                                 BType::PROD(pr.rhs, pair.second.getType()));
                std::vector<Pred> vec;
                vec.push_back(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp_1), set.lhs.copy()));
                vec.push_back(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp_2), set.rhs.copy()));
                ppTrans(str2, ctx, Pred::makeExists(vars, Pred::makeConjunction(std::move(vec))), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Surcharge: {
                BType ty_lhs = lhs.getType();
                assert(ty_lhs.getKind() == BType::Kind::ProductType);
                auto &pr = ty_lhs.toProductType();
                ppTrans_mem(str, ctx, lhs, set.rhs, used_ids);
                str << " | ";
                ppTrans_mem(str, ctx, lhs, // FIXME
                            Expr::makeBinaryExpr(
                                    Expr::BinaryOp::Domain_Subtraction,
                                    Expr::makeUnaryExpr(Expr::UnaryOp::Domain, set.rhs.copy(), BType::POW(pr.lhs)),
                                    set.lhs.copy(),
                                    set.lhs.getType()), used_ids);
                return;
            }
            case Expr::BinaryOp::Relations: {
                Expr prod = Expr::makeBinaryExpr(Expr::BinaryOp::Cartesian_Product, set.lhs.copy(), set.rhs.copy(),
                                                 lhs.getType());
                return ppTrans(str, ctx,
                               Pred::makeExprComparison(Pred::ComparisonOp::Subset, lhs.copy(), std::move(prod)),
                               used_ids);
            }
            case Expr::BinaryOp::Tail_Insertion: {
                std::vector<Expr> vec;
                vec.push_back(set.rhs.copy());
                Expr sing = Expr::makeNaryExpr(Expr::NaryOp::Sequence, std::move(vec), ty_rhs);
                Expr new_rhs = Expr::makeBinaryExpr(Expr::BinaryOp::Concatenation, set.lhs.copy(), std::move(sing),
                                                    ty_rhs);
                return ppTrans_mem(str, ctx, lhs, new_rhs, used_ids);
            }
            case Expr::BinaryOp::Domain_Subtraction: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), set.rhs.copy()));
                vec.push_back(Pred::makeNegation(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair.first),
                                                 set.lhs.copy())));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Domain_Restriction: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), set.rhs.copy()));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair.first),
                                                       set.lhs.copy()));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Direct_Product: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto e_fg = splitPair(eqs, lhs);
                auto f_g = splitPair(eqs, e_fg.second);
                auto ty_e = e_fg.first.getType();
                auto ty_f = f_g.first.getType();
                auto ty_g = f_g.second.getType();
                Expr mp1 = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, e_fg.first.copy(), std::move(f_g.first),
                                                BType::PROD(ty_e, ty_f));
                Expr mp2 = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(e_fg.first), std::move(f_g.second),
                                                BType::PROD(ty_e, ty_g));
                ppTrans_mem(str2, ctx, mp1, set.lhs, used_ids);
                str2 << " & ";
                ppTrans_mem(str2, ctx, mp2, set.rhs, used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Parallel_Product: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto ef_gh = splitPair(eqs, lhs);
                auto e_f = splitPair(eqs, ef_gh.first);
                auto g_h = splitPair(eqs, ef_gh.second);
                auto ty_e = e_f.first.getType();
                auto ty_f = e_f.second.getType();
                auto ty_g = g_h.first.getType();
                auto ty_h = g_h.second.getType();
                Expr mp1 = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(e_f.first), std::move(g_h.first),
                                                BType::PROD(ty_e, ty_g));
                Expr mp2 = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(e_f.second), std::move(g_h.second),
                                                BType::PROD(ty_f, ty_h));
                ppTrans_mem(str2, ctx, mp1, set.lhs, used_ids);
                str2 << " & ";
                ppTrans_mem(str2, ctx, mp2, set.rhs, used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Union: {
                // E.g. x : a\/b --> x : a | x : b
                ppTrans_mem(str, ctx, lhs, set.lhs, used_ids);
                str << " | ";
                ppTrans_mem(str, ctx, lhs, set.rhs, used_ids);
                str << ""; // FIXME Look into this
                return;
            }
            case Expr::BinaryOp::Tail_Restriction: // x : a\|/b avec x=(x1,x2) --> x1+b |-> x2 : a
            {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                Expr add = Expr::makeBinaryExpr(Expr::BinaryOp::IAddition, std::move(pair.first),
                                                set.rhs.copy(), BType::INT);
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(add), std::move(pair.second),
                                               lhs.getType());
                Pred p = Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp), set.lhs.copy());
                ppTrans(str2, ctx, p, used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Concatenation:
                // x: a^b avec x=(x1,x2) --> x:a ou (x1-size(a),x2):b
                // (x1,x2): a^b --> exists s. s=a & ((x1,x2):s ou (x1-size(s),x2):b)
            {
                LocalEquations eqs;
                std::ostringstream str2;
                std::vector<TypedVar> vars;
                Expr s = getFreshVars(set.lhs.getType(), vars);
                auto pair = splitPair(eqs, lhs);
                Expr diff = Expr::makeBinaryExpr(Expr::BinaryOp::ISubtraction, std::move(pair.first),
                                                 Expr::makeUnaryExpr(Expr::UnaryOp::Size, s.copy(), BType::INT),
                                                 BType::INT);
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(diff), std::move(pair.second),
                                               lhs.getType());
                std::vector<Pred> disj;
                disj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), s.copy()));
                disj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp), set.rhs.copy()));
                std::vector<Pred> conj;
                conj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, s.copy(), set.lhs.copy()));
                conj.push_back(Pred::makeDisjunction(std::move(disj)));
                ppTrans(str2, ctx, Pred::makeExists(vars, Pred::makeConjunction(std::move(conj))), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Range_Restriction: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), set.lhs.copy()));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair.second),
                                                       set.rhs.copy()));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Range_Subtraction: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), set.lhs.copy()));
                vec.push_back(Pred::makeNegation(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair.second),
                                                 set.rhs.copy())));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Image: {
                BType ty_f = set.lhs.getType();
                assert(ty_f.getKind() == BType::Kind::PowerType);
                BType prod = ty_f.toPowerType().content;
                assert(prod.getKind() == BType::Kind::ProductType);
                auto pr = prod.toProductType();
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(pr.lhs, vars);
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, x.copy(), lhs.copy(),
                                               BType::PROD(pr.lhs, lhs.getType()));
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(x), set.rhs.copy()));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp), set.lhs.copy()));
                return ppTrans(str, ctx, Pred::makeExists(vars, Pred::makeConjunction(std::move(vec))), used_ids);
            }
            case Expr::BinaryOp::Application:
                assert(false);

            case Expr::BinaryOp::First_Projection: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair1 = splitPair(eqs, lhs);
                auto pair2 = splitPair(eqs, pair1.first);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, pair2.first.copy(),
                                                       std::move(pair1.second)));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair2.first),
                                                       set.lhs.copy()));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair2.second),
                                                       set.rhs.copy()));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Second_Projection: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair1 = splitPair(eqs, lhs);
                auto pair2 = splitPair(eqs, pair1.first);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, pair2.second.copy(),
                                                       std::move(pair1.second)));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair2.first),
                                                       set.lhs.copy()));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(pair2.second),
                                                       set.rhs.copy()));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Iteration: {
                LocalEquations eqs;
                std::ostringstream str2;
                std::string id_rel = ctx.nameSimpleExpression(set.lhs, eqs, used_ids);
                //assert(ty_rhs.getKind() == BType::Kind::PowerType);
                str2 << "(" << ctx.registerIterate(set.lhs.getType(), used_ids) << "(";
                toIdentList(str2, ctx, eqs, lhs, used_ids);
                str2 << " " << id_rel << ", ";
                ppTrans(str2, ctx, eqs, set.rhs, used_ids);
                str2 << "))";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::BinaryOp::Const:
            case Expr::BinaryOp::Rank:
            case Expr::BinaryOp::Father:
            case Expr::BinaryOp::Subtree:
            case Expr::BinaryOp::Arity:
                throw ppTransException("ppTrans_mem: tree constructs not implemented.");

            case Expr::BinaryOp::RExponentiation:
            case Expr::BinaryOp::RAddition:
            case Expr::BinaryOp::RSubtraction:
            case Expr::BinaryOp::RDivision:
            case Expr::BinaryOp::RMultiplication:
            case Expr::BinaryOp::FAddition:
            case Expr::BinaryOp::FSubtraction:
            case Expr::BinaryOp::FDivision:
            case Expr::BinaryOp::FMultiplication:
            case Expr::BinaryOp::Mapplet:
            case Expr::BinaryOp::IExponentiation:
            case Expr::BinaryOp::IAddition:
            case Expr::BinaryOp::ISubtraction:
            case Expr::BinaryOp::IDivision:
            case Expr::BinaryOp::IMultiplication:
            case Expr::BinaryOp::Modulo:
                throw ppTransException("ppTrans_mem: type error (1).");
        }
        assert(false); // unreachable
    }

    void ppTrans_mem(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr::NaryExpr &rhs,
                     std::set<std::string> &used_ids) {
        switch (rhs.op) {
            case Expr::NaryOp::Sequence: {
                std::vector<Pred> disj;
                int cpt = 1;
                for (auto &e: rhs.vec) {
                    Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, Expr::makeInteger(std::to_string(cpt)),
                                                   e.copy(), lhs.getType());
                    disj.push_back(
                            Pred::makeExprComparison(Pred::ComparisonOp::Equality, lhs.copy(), std::move(mp))); // FIXME
                    ++cpt;
                }
                return ppTrans(str, ctx, Pred::makeDisjunction(std::move(disj)), used_ids);
            }
            case Expr::NaryOp::Set: {
                std::vector<Pred> disj;
                for (auto &e: rhs.vec)
                    disj.push_back(
                            Pred::makeExprComparison(Pred::ComparisonOp::Equality, lhs.copy(), e.copy())); // FIXME
                return ppTrans(str, ctx, Pred::makeDisjunction(std::move(disj)), used_ids);
            }
        }
        assert(false); // unreachable
    }

    void ppTrans_mem(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr::QuantifiedExpr &rhs,
                     std::set<std::string> &used_ids) {
        switch (rhs.op) {
            case Expr::QuantifiedOp::Lambda: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto lhs_pair = splitPair(eqs, lhs);
                std::map<VarName, Expr> subst;
                assert(!rhs.vars.empty());
                Expr tmp = std::move(lhs_pair.first);
                for (size_t i = rhs.vars.size() - 1; 0 < i; --i) {
                    auto pair = splitPair(eqs, tmp);
                    subst[rhs.vars[i].name] = std::move(pair.second);
                    tmp = std::move(pair.first);
                }
                subst[rhs.vars[0].name] = std::move(tmp);
                Pred cond = rhs.cond.copy();
                cond.subst(subst);
                Expr body = rhs.body.copy();
                body.subst(subst);
                std::vector<Pred> vec;
                vec.push_back(std::move(cond));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, std::move(lhs_pair.second),
                                                       std::move(body)));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::QuantifiedOp::Intersection:
                return ppTrans(str, ctx, Pred::makeForall(rhs.vars,
                                                          Pred::makeImplication(
                                                                  rhs.cond.copy(),
                                                                  Pred::makeExprComparison(
                                                                          Pred::ComparisonOp::Membership, lhs.copy(),
                                                                          rhs.body.copy()))), used_ids);
            case Expr::QuantifiedOp::Union: {
                std::vector<Pred> vec;
                vec.push_back(rhs.cond.copy());
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), rhs.body.copy()));
                return ppTrans(str, ctx, Pred::makeExists(rhs.vars, Pred::makeConjunction(std::move(vec))), used_ids);
            }
            case Expr::QuantifiedOp::ISum:
            case Expr::QuantifiedOp::IProduct:
            case Expr::QuantifiedOp::RSum:
            case Expr::QuantifiedOp::RProduct:
                throw ppTransException("ppTrans_mem: type error.");
        }
        assert(false); // unreachable
    }

    void ppTrans_mem(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr::UnaryExpr &rhs,
                     std::set<std::string> &used_ids) {
        BType ty_rhs = BType::POW(lhs.getType());
        switch (rhs.op) {
            case Expr::UnaryOp::Domain: {
                BType ty_content = rhs.content.getType();
                assert(ty_content.getKind() == BType::Kind::PowerType);
                BType ty_mp = ty_content.toPowerType().content;
                assert(ty_mp.getKind() == BType::Kind::ProductType);
                auto &pr = ty_mp.toProductType();
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(pr.rhs, vars);
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, lhs.copy(), std::move(x), ty_mp);
                Pred p = Pred::makeExists(vars, Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp),
                                                                         rhs.content.copy()));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Range: {
                BType ty_content = rhs.content.getType();
                assert(ty_content.getKind() == BType::Kind::PowerType);
                BType ty_mp = ty_content.toPowerType().content;
                assert(ty_mp.getKind() == BType::Kind::ProductType);
                auto &pr = ty_mp.toProductType();
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(pr.lhs, vars);
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(x), lhs.copy(), ty_mp);
                Pred p = Pred::makeExists(vars, Pred::makeExprComparison(Pred::ComparisonOp::Membership, std::move(mp),
                                                                         rhs.content.copy()));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Subsets: {
                const BType &ty_lhs = lhs.getType();
                assert(ty_lhs.getKind() == BType::Kind::PowerType);
                BType ty_x = ty_lhs.toPowerType().content;
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(ty_x, vars);
                Pred p = Pred::makeForall(vars, Pred::makeImplication(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, x.copy(), lhs.copy()),
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, x.copy(), rhs.content.copy())));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Non_Empty_Subsets: {
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(),
                                                       Expr::makeUnaryExpr(Expr::UnaryOp::Subsets, rhs.content.copy(),
                                                                           ty_rhs)));
                vec.push_back(
                        Pred::makeNegation(Pred::makeExprComparison(Pred::ComparisonOp::Equality, lhs.copy(), // FIXME
                                                                    Expr::makeEmptySet(lhs.getType()))));
                return ppTrans(str, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
            }
            case Expr::UnaryOp::Finite_Subsets: {
                std::vector<TypedVar> vars;
                Expr n = getFreshVars(BType::INT, vars);
                Expr itvl = Expr::makeBinaryExpr(
                        Expr::BinaryOp::Interval,
                        Expr::makeInteger("1"),
                        std::move(n),
                        BType::POW(BType::INT));
                BType ty_f = BType::POW(BType::PROD(BType::INT, lhs.getType().toPowerType().content));
                Expr f = getFreshVars(ty_f, vars);
                Pred isFinite = Pred::makeExists(vars, Pred::makeExprComparison(
                        Pred::ComparisonOp::Membership,
                        std::move(f),
                        Expr::makeBinaryExpr(Expr::BinaryOp::Total_Bijections, std::move(itvl), lhs.copy(),
                                             BType::POW(ty_f))
                ));
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), // FIXME
                                                       Expr::makeUnaryExpr(Expr::UnaryOp::Subsets, rhs.content.copy(),
                                                                           ty_rhs)));
                vec.push_back(std::move(isFinite));
                return ppTrans(str, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
            }
            case Expr::UnaryOp::Non_Empty_Finite_Subsets: {
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(),
                                                       Expr::makeUnaryExpr(Expr::UnaryOp::Finite_Subsets,
                                                                           rhs.content.copy(), ty_rhs)));
                vec.push_back(
                        Pred::makeNegation(Pred::makeExprComparison(Pred::ComparisonOp::Equality, lhs.copy(), // FIXME
                                                                    Expr::makeEmptySet(lhs.getType()))));
                return ppTrans(str, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
            }
            case Expr::UnaryOp::Union: {
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(BType::POW(lhs.getType()), vars);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, x.copy(), rhs.content.copy()));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), x.copy()));
                return ppTrans(str, ctx, Pred::makeExists(vars, Pred::makeConjunction(std::move(vec))), used_ids);
            }
            case Expr::UnaryOp::Intersection: {
                std::vector<TypedVar> vars;
                Expr x = getFreshVars(BType::POW(lhs.getType()), vars);
                return ppTrans(str, ctx, Pred::makeForall(vars, Pred::makeImplication(
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, x.copy(), rhs.content.copy()),
                        Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(), x.copy()))), used_ids);
            }
            case Expr::UnaryOp::Sequences: // e1 : seq(s) --> exists n. e1 : (1..n --> s)
            {
                std::vector<TypedVar> vars;
                const BType &ty_lhs = lhs.getType();
                Expr n = getFreshVars(BType::INT, vars);
                Expr itvl = Expr::makeBinaryExpr(Expr::BinaryOp::Interval, Expr::makeInteger("1"), std::move(n),
                                                 BType::POW(BType::INT));
                Pred p = Pred::makeExists(vars, Pred::makeExprComparison(
                        Pred::ComparisonOp::Membership,
                        lhs.copy(),
                        Expr::makeBinaryExpr(Expr::BinaryOp::Total_Functions, std::move(itvl), rhs.content.copy(),
                                             BType::POW(ty_lhs))));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Non_Empty_Sequences: {
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(),
                                                       Expr::makeUnaryExpr(Expr::UnaryOp::Sequences, rhs.content.copy(),
                                                                           ty_rhs)));
                vec.push_back(Pred::makeNegation(Pred::makeExprComparison(Pred::ComparisonOp::Equality, lhs.copy(),
                                                                          Expr::makeEmptySet(lhs.getType())))); // FIXME
                Pred p = Pred::makeConjunction(std::move(vec));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Injective_Sequences:  // e1 : iseq(s) --> exists n. e1 : (1..n >-> s)
            {
                std::vector<TypedVar> vars;
                const BType &ty_lhs = lhs.getType();
                Expr n = getFreshVars(BType::INT, vars);
                Expr itvl = Expr::makeBinaryExpr(Expr::BinaryOp::Interval, Expr::makeInteger("1"), std::move(n),
                                                 BType::POW(BType::INT));
                Pred p = Pred::makeExists(vars, Pred::makeExprComparison(
                        Pred::ComparisonOp::Membership,
                        lhs.copy(),
                        Expr::makeBinaryExpr(Expr::BinaryOp::Total_Injections, std::move(itvl), rhs.content.copy(),
                                             BType::POW(ty_lhs))));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Non_Empty_Injective_Sequences: {
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(),
                                                       Expr::makeUnaryExpr(Expr::UnaryOp::Injective_Sequences,
                                                                           rhs.content.copy(), ty_rhs)));
                vec.push_back(Pred::makeNegation(Pred::makeExprComparison(Pred::ComparisonOp::Equality, lhs.copy(),
                                                                          Expr::makeEmptySet(lhs.getType())))); // FIXME
                Pred p = Pred::makeConjunction(std::move(vec));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Inverse: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                BType ty_mp = BType::PROD(pair.second.getType(), pair.first.getType());
                Expr mp = Expr::makeBinaryExpr(Expr::BinaryOp::Mapplet, std::move(pair.second), std::move(pair.first),
                                               ty_mp);
                ppTrans_mem(str2, ctx, mp, rhs.content, used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::UnaryOp::Permutations: {
                std::vector<TypedVar> vars;
                const BType &ty_lhs = lhs.getType();
                Expr n = getFreshVars(BType::INT, vars);
                Expr itvl = Expr::makeBinaryExpr(Expr::BinaryOp::Interval, Expr::makeInteger("1"), std::move(n),
                                                 BType::POW(BType::INT));
                Pred p = Pred::makeExists(vars, Pred::makeExprComparison(
                        Pred::ComparisonOp::Membership,
                        lhs.copy(),
                        Expr::makeBinaryExpr(Expr::BinaryOp::Total_Bijections, std::move(itvl), rhs.content.copy(),
                                             BType::POW(ty_lhs))));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::First:
            case Expr::UnaryOp::Last:
                assert(false);
            case Expr::UnaryOp::Identity: {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, pair.first.copy(),
                                                       std::move(pair.second)));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, pair.first.copy(),
                                                       rhs.content.copy()));
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(vec)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::UnaryOp::Closure: // lhs : closure(u.content) --> exists n. n>=0 and lhs:iterate(u.content,n)
            {
                std::vector<TypedVar> vars;
                Expr n = getFreshVars(BType::INT, vars);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Ige, n.copy(), Expr::makeInteger("0")));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(),
                                                       Expr::makeBinaryExpr(Expr::BinaryOp::Iteration,
                                                                            rhs.content.copy(), n.copy(), ty_rhs)));
                Pred p = Pred::makeExists(vars, Pred::makeConjunction(std::move(vec)));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Transitive_Closure: {
                std::vector<TypedVar> vars;
                Expr n = getFreshVars(BType::INT, vars);
                std::vector<Pred> vec;
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Ige, n.copy(), Expr::makeInteger("1")));
                vec.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, lhs.copy(),
                                                       Expr::makeBinaryExpr(Expr::BinaryOp::Iteration,
                                                                            rhs.content.copy(), n.copy(), ty_rhs)));
                Pred p = Pred::makeExists(vars, Pred::makeConjunction(std::move(vec)));
                return ppTrans(str, ctx, p, used_ids);
            }
            case Expr::UnaryOp::Tail: // tail(s) = s \|/ 1
            {
                Expr e = Expr::makeBinaryExpr(Expr::BinaryOp::Tail_Restriction, rhs.content.copy(),
                                              Expr::makeInteger("1"), ty_rhs);
                return ppTrans_mem(str, ctx, lhs, e, used_ids);
            }
            case Expr::UnaryOp::Front: // front(seq) = seq - { (size(seq)|->seq(size(seq))) }
            {
                BType seq_typ = rhs.content.getType();
                assert(seq_typ.getKind() == BType::Kind::PowerType);
                BType prod = seq_typ.toPowerType().content;
                assert(prod.getKind() == BType::Kind::ProductType);
                BType ty_elt = prod.toProductType().rhs;
                Expr sz = Expr::makeUnaryExpr(Expr::UnaryOp::Size, rhs.content.copy(), BType::INT);
                std::vector<Expr> vec;
                vec.push_back(Expr::makeBinaryExpr(
                        Expr::BinaryOp::Mapplet, std::move(sz),
                        Expr::makeBinaryExpr(Expr::BinaryOp::Application, rhs.content.copy(), sz.copy(),
                                             ty_elt), // FIXME
                        prod));
                Expr last = Expr::makeNaryExpr(Expr::NaryOp::Set, std::move(vec), seq_typ);
                Expr front = Expr::makeBinaryExpr(Expr::BinaryOp::Set_Difference, rhs.content.copy(), last.copy(),
                                                  seq_typ);
                return ppTrans_mem(str, ctx, lhs, front, used_ids);
            }
            case Expr::UnaryOp::Reverse: {
                std::vector<TypedVar> vars;
                Expr i = getFreshVars(BType::INT, vars);
                Expr size = Expr::makeUnaryExpr(Expr::UnaryOp::Size, rhs.content.copy(), BType::INT);
                Expr itvl = Expr::makeBinaryExpr(Expr::BinaryOp::Interval, Expr::makeInteger("1"), size.copy(),
                                                 BType::POW(BType::INT));
                Pred cond = Pred::makeExprComparison(Pred::ComparisonOp::Membership, i.copy(), std::move(itvl));
                Expr j = Expr::makeBinaryExpr(
                        Expr::BinaryOp::IAddition,
                        Expr::makeBinaryExpr(Expr::BinaryOp::ISubtraction, std::move(size), std::move(i), BType::INT),
                        Expr::makeInteger("1"), BType::INT);
                Expr body = Expr::makeBinaryExpr(Expr::BinaryOp::Application, rhs.content.copy(), std::move(j),
                                                 ty_rhs.toPowerType().content.toProductType().rhs); // FIXME
                Expr rev = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::Lambda, vars, std::move(cond), std::move(body),
                                                    ty_rhs);
                return ppTrans_mem(str, ctx, lhs, rev, used_ids);
            }
            case Expr::UnaryOp::Concatenation: {
                // (x,y) : conc(c) ---> exists i,j,s. s=c & i:1..size(s) & j:1..size(s(i)) & x = sum(k).( k:1..(i-1) | size(s(k)) ) & y = s(i)(j)
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                std::vector<TypedVar> vars;
                Expr i = getFreshVars(BType::INT, vars);
                Expr j = getFreshVars(BType::INT, vars);
                Expr s = getFreshVars(rhs.content.getType(), vars);
                std::vector<Pred> conj;
                Expr one = Expr::makeInteger("1");
                conj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, s.copy(), rhs.content.copy()));
                Expr size_s = Expr::makeUnaryExpr(Expr::UnaryOp::Size, s.copy(), BType::INT);
                //BType ty_elt = lhs.getType().toProductType().rhs;
                conj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, i.copy(),
                                                        Expr::makeBinaryExpr(Expr::BinaryOp::Interval, one.copy(),
                                                                             std::move(size_s), BType::POW_INT)));
                Expr size_s_i = Expr::makeUnaryExpr(Expr::UnaryOp::Size,
                                                    Expr::makeBinaryExpr(Expr::BinaryOp::Application, s.copy(),
                                                                         i.copy(), ty_rhs), BType::INT);
                conj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Membership, j.copy(),
                                                        Expr::makeBinaryExpr(Expr::BinaryOp::Interval, one.copy(),
                                                                             std::move(size_s_i), BType::POW_INT)));
                std::vector<TypedVar> vk;
                Expr k = getFreshVars(BType::INT, vk);
                Expr size_s_k = Expr::makeUnaryExpr(Expr::UnaryOp::Size,
                                                    Expr::makeBinaryExpr(Expr::BinaryOp::Application, s.copy(),
                                                                         k.copy(), ty_rhs), BType::INT);
                Expr i_minus_one = Expr::makeBinaryExpr(Expr::BinaryOp::ISubtraction, i.copy(), one.copy(), BType::INT);
                Pred mem = Pred::makeExprComparison(Pred::ComparisonOp::Membership, k.copy(),
                                                    Expr::makeBinaryExpr(Expr::BinaryOp::Interval, one.copy(),
                                                                         std::move(i_minus_one), BType::POW_INT));
                Expr sum = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::ISum, vk, std::move(mem), std::move(size_s_k),
                                                    BType::INT);
                conj.push_back(
                        Pred::makeExprComparison(Pred::ComparisonOp::Equality, pair.first.copy(), std::move(sum)));
                Expr s_i = Expr::makeBinaryExpr(Expr::BinaryOp::Application, s.copy(), i.copy(), ty_rhs);
                conj.push_back(Pred::makeExprComparison(Pred::ComparisonOp::Equality, pair.second.copy(),
                                                        Expr::makeBinaryExpr(Expr::BinaryOp::Application,
                                                                             std::move(s_i), j.copy(),
                                                                             pair.second.getType())));
                ppTrans(str2, ctx, Pred::makeExists(vars, Pred::makeConjunction(std::move(conj))), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::UnaryOp::Rel: // (x,y) : rel(f) ---> x:dom(f) and y:f(x)
            {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                Expr dom = Expr::makeUnaryExpr(Expr::UnaryOp::Domain, rhs.content.copy(),
                                               BType::POW(pair.first.getType()));
                Expr app = Expr::makeBinaryExpr(Expr::BinaryOp::Application, rhs.content.copy(),
                                                pair.first.copy(), // FIXME
                                                BType::POW(pair.second.getType()));
                str2 << "(";
                ppTrans_mem(str2, ctx, pair.first, dom, used_ids);
                str2 << " & ";
                ppTrans_mem(str2, ctx, pair.second, app, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::UnaryOp::Fnc: {
                std::vector<TypedVar> vars;
                Expr i = getFreshVars(rhs.content.getType().toPowerType().content.toProductType().lhs, vars);
                Expr dom = Expr::makeUnaryExpr(Expr::UnaryOp::Domain, rhs.content.copy(),
                                               BType::POW(i.getType()));
                Pred cond = Pred::makeExprComparison(Pred::ComparisonOp::Membership, i.copy(), std::move(dom));
                std::vector<Expr> vec;
                vec.push_back(i.copy());
                Expr body = Expr::makeBinaryExpr(
                        Expr::BinaryOp::Image,
                        rhs.content.copy(), // FIXME
                        Expr::makeNaryExpr(Expr::NaryOp::Set, std::move(vec), BType::POW(i.getType())),
                        BType::POW(rhs.content.getType().toPowerType().content.toProductType().rhs));
                Expr lam = Expr::makeQuantifiedExpr(Expr::QuantifiedOp::Lambda, vars, std::move(cond), std::move(body),
                                                    ty_rhs);
                return ppTrans_mem(str, ctx, lhs, lam, used_ids);
            }
            case Expr::UnaryOp::Tree:
            case Expr::UnaryOp::Btree:
            case Expr::UnaryOp::Top:
            case Expr::UnaryOp::Sons:
            case Expr::UnaryOp::Prefix:
            case Expr::UnaryOp::Postfix:
            case Expr::UnaryOp::Sizet:
            case Expr::UnaryOp::Mirror:
            case Expr::UnaryOp::Left:
            case Expr::UnaryOp::Right:
            case Expr::UnaryOp::Infix:
            case Expr::UnaryOp::Bin:
                throw ppTransException("ppTrans_mem: tree constructs not implemented.");

            case Expr::UnaryOp::IMinus:
            case Expr::UnaryOp::IMaximum:
            case Expr::UnaryOp::IMinimum:
            case Expr::UnaryOp::Cardinality:
            case Expr::UnaryOp::Size:
            case Expr::UnaryOp::Real:
            case Expr::UnaryOp::Floor:
            case Expr::UnaryOp::Ceiling:
            case Expr::UnaryOp::RMinimum:
            case Expr::UnaryOp::RMaximum:
            case Expr::UnaryOp::RMinus:
                throw ppTransException("ppTrans_mem: type error.");
        }
        assert(false); // unreachable
    }

    // Convert a membership predicate [lhs]:[rhs] into a smtlib string
    void ppTrans_mem(std::ostringstream &str, Context &ctx, const Expr &lhs, const Expr &rhs,
                     std::set<std::string> &used_ids) {
        //assert(BType::weak_eq(BType::POW(lhs.getType()),rhs.getType()));

        switch (rhs.getTag()) {
            case Expr::EKind::BinaryExpr: {
                auto &b = rhs.toBinaryExpr();
                if (b.op == Expr::BinaryOp::Application) {
                    LocalEquations eqs;
                    std::ostringstream str2;
                    std::string id = ctx.nameSimpleExpression(rhs, eqs, used_ids);
                    makeMem(str2, ctx, eqs, lhs, id, rhs.getType(), used_ids);
                    return add_local_defs(str, ctx, eqs, str2, used_ids);
                } else {
                    return ppTrans_mem(str, ctx, lhs, rhs.toBinaryExpr(), used_ids);
                }
            }
            case Expr::EKind::NaryExpr:
                return ppTrans_mem(str, ctx, lhs, rhs.toNaryExpr(), used_ids);
            case Expr::EKind::EmptySet:
                str << "$false";
                return;
            case Expr::EKind::INTEGER:
            case Expr::EKind::STRING:
            case Expr::EKind::BOOL:
            case Expr::EKind::REAL:
            case Expr::EKind::FLOAT:
                str << "$true";
                return;
            case Expr::EKind::NATURAL: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greatereq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", 0)";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::NATURAL1: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greatereq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", 1)";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::INT: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "($greatereq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", min_int) & $lesseq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids); // FIXME
                str2 << ", max_int))";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::NAT: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "($greatereq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids); // FIXME
                str2 << ", 0) & $lesseq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", max_int))";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::NAT1: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "($greater(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", 0) & $lesseq(";
                ppTrans(str2, ctx, eqs, lhs, used_ids);
                str2 << ", max_int))";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::Id: {
                LocalEquations eqs;
                std::ostringstream str2;
                makeMem(str2, ctx, eqs, lhs, ctx.registerId(rhs.getId(), rhs.getType(), used_ids), rhs.getType(),
                        used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::QuantifiedExpr:
                return ppTrans_mem(str, ctx, lhs, rhs.toQuantiedExpr(), used_ids);

            case Expr::EKind::QuantifiedSet: {
                auto &q = rhs.toQuantifiedSet();
                std::map<VarName, Expr> subst;
                assert(!q.vars.empty());
                Expr tmp = lhs.copy();
                LocalEquations eqs;
                std::ostringstream str2;
                for (size_t i = q.vars.size() - 1; i > 0; i--) {
                    auto pair = splitPair(eqs, tmp);
                    subst[q.vars[i].name] = std::move(pair.second);
                    tmp = std::move(pair.first);
                }
                subst[q.vars[0].name] = std::move(tmp);
                Pred cond = q.cond.copy();
                cond.subst(subst);
                ppTrans(str2, ctx, cond, used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::UnaryExpr: {
                auto &u = rhs.toUnaryExpr();
                if (u.op == Expr::UnaryOp::Last || u.op == Expr::UnaryOp::First) {
                    LocalEquations eqs;
                    std::ostringstream str2;
                    std::string id = ctx.nameSimpleExpression(rhs, eqs, used_ids);
                    makeMem(str2, ctx, eqs, lhs, id, rhs.getType(), used_ids);
                    return add_local_defs(str, ctx, eqs, str2, used_ids);
                } else {
                    return ppTrans_mem(str, ctx, lhs, rhs.toUnaryExpr(), used_ids);
                }
            }
            case Expr::EKind::TernaryExpr:
                throw ppTransException("ppTrans_mem: tree constructs not implemented.");
            case Expr::EKind::Record_Field_Access: {
                LocalEquations eqs;
                std::ostringstream str2;
                ppTrans_mem(str2, ctx, lhs, getRecordField(eqs, rhs.toRecordAccess()), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::Struct: {
                auto &s = rhs.toStructExpr();
                LocalEquations eqs;
                std::ostringstream str2;
                auto lhs_fields = splitRecord(eqs, lhs);
                std::vector<Pred> conj;
                assert(lhs_fields.size() == s.fields.size());
                for (size_t i = 0; i < s.fields.size(); i++) {
                    assert(lhs_fields[i].first == s.fields[i].first);
                    conj.push_back(
                            Pred::makeExprComparison(
                                    Pred::ComparisonOp::Membership,
                                    lhs_fields[i].second.copy(),
                                    s.fields[i].second.copy()));
                }
                ppTrans(str2, ctx, Pred::makeConjunction(std::move(conj)), used_ids);
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::Predecessor : {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                ppTrans(str2, ctx, eqs, pair.second, used_ids);
                str2 << " = $difference(";
                ppTrans(str2, ctx, eqs, pair.first, used_ids);
                str2 << ", 1)";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::Successor : {
                LocalEquations eqs;
                std::ostringstream str2;
                auto pair = splitPair(eqs, lhs);
                ppTrans(str2, ctx, eqs, pair.second, used_ids);
                str2 << " = $sum(";
                ppTrans(str2, ctx, eqs, pair.first, used_ids);
                str2 << ", 1)";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Expr::EKind::Record:
            case Expr::EKind::Record_Field_Update:
            case Expr::EKind::BooleanExpr:
            case Expr::EKind::StringLiteral:
            case Expr::EKind::RealLiteral:
            case Expr::EKind::IntegerLiteral:
            case Expr::EKind::TRUE:
            case Expr::EKind::FALSE:
            case Expr::EKind::MinInt:
            case Expr::EKind::MaxInt:
                throw ppTransException("ppTrans_mem: type error.");
        }
        assert(false); // unreachable
    }

    void
    ppTrans(std::ostringstream &str, Context &ctx, const Pred::ExprComparison &cmp, std::set<std::string> &used_ids) {
        switch (cmp.op) {
            case Pred::ComparisonOp::Subset: {
                BType ty_lhs = cmp.lhs.getType();
                return ppTrans_mem(str, ctx, cmp.lhs,
                                   Expr::makeUnaryExpr(Expr::UnaryOp::Subsets, cmp.rhs.copy(), BType::POW(ty_lhs)),
                                   used_ids);
            }
            case Pred::ComparisonOp::Strict_Subset: {
                BType ty_lhs = cmp.lhs.getType();
                str << "(";
                ppTrans_mem(str, ctx, cmp.lhs,
                            Expr::makeUnaryExpr(Expr::UnaryOp::Subsets, cmp.rhs.copy(), BType::POW(ty_lhs)), used_ids);
                str << " & (~(";
                ppTrans_eq(str, ctx, cmp.lhs, cmp.rhs, used_ids); // FIXME
                str << ")))";
                return;
            }
            case Pred::ComparisonOp::Membership:
                return ppTrans_mem(str, ctx, cmp.lhs, cmp.rhs, used_ids);
            case Pred::ComparisonOp::Equality:
                return ppTrans_eq(str, ctx, cmp.lhs, cmp.rhs, used_ids);
            case Pred::ComparisonOp::Ige: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greatereq(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Igt: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greater(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Ile: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$lesseq(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Ilt: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$less(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Rge: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greatereq(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Rgt: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greater(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Rle: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$lesseq(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Rlt: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$less(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Fge: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greatereq(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Fgt: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$greater(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Fle: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$lesseq(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << ", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
            case Pred::ComparisonOp::Flt: {
                LocalEquations eqs;
                std::ostringstream str2;
                str2 << "$less(";
                ppTrans(str2, ctx, eqs, cmp.lhs, used_ids);
                str2 << +", ";
                ppTrans(str2, ctx, eqs, cmp.rhs, used_ids);
                str2 << ")";
                return add_local_defs(str, ctx, eqs, str2, used_ids);
            }
        }
        assert(false); // unreachable
    }

    // Convert a predicate into an TPTP string
    void ppTrans(std::ostringstream &str, Context &ctx, const Pred &p, std::set<std::string> &used_ids) {
        switch (p.getTag()) {
            case Pred::PKind::True:
                str << "$true";
                return;
            case Pred::PKind::False:
                str << "$false";
                return;
            case Pred::PKind::Implication: {
                auto &b = p.toImplication();
                str << "((";
                ppTrans(str, ctx, b.lhs, used_ids);
                str << ") => (";
                ppTrans(str, ctx, b.rhs, used_ids);
                str << "))";
                return;
            }
            case Pred::PKind::Equivalence: {
                // Equivalence is not supported by TPTP, so we have to use bi-implication
                auto &b = p.toEquivalence();
                str << "(";
                ppTrans(str, ctx, b.lhs, used_ids);
                str << ") <=> (";
                ppTrans(str, ctx, b.rhs, used_ids);
                str << ")";
                return;
            }
            case Pred::PKind::ExprComparison:
                return ppTrans(str, ctx, p.toExprComparison(), used_ids);
            case Pred::PKind::Negation:
                str << "(~(";
                ppTrans(str, ctx, p.toNegation().operand, used_ids);
                str << "))";
                return;
            case Pred::PKind::Conjunction: {
                auto &n = p.toConjunction();
                if (n.operands.empty()) {
                    str << "$true";
                    return;
                } else if (n.operands.size() == 1) {
                    return ppTrans(str, ctx, n.operands.at(0), used_ids);
                } else {
                    str << "(";
                    for (size_t i = 0; i < n.operands.size(); i++) {
                        auto &q = n.operands.at(i);
                        ppTrans(str, ctx, q, used_ids);
                        if (i < n.operands.size() - 1) {
                            str << " & ";
                        }
                    }
                    str << ")";
                    return;
                }
            }
            case Pred::PKind::Disjunction: {
                auto &n = p.toDisjunction();
                if (n.operands.empty()) {
                    str << "$false";
                    return;
                } else if (n.operands.size() == 1) {
                    return ppTrans(str, ctx, n.operands.at(0), used_ids);
                } else {
                    str << "(";
                    for (size_t i = 0; i < n.operands.size(); i++) {
                        auto &q = n.operands.at(i);
                        ppTrans(str, ctx, q, used_ids);
                        if (i < n.operands.size() - 1) {
                            str << " | ";
                        }
                    }
                    str << ")";
                    return;
                }
            }
            case Pred::PKind::Forall: {
                auto &q = p.toForall();
                str << "! [";
                for (size_t i = 0; i < q.vars.size(); i++) {
                    // Bounded variables
                    const auto &v = q.vars.at(i);
                    str << localVarNameToString(v.name) << " : " << ppTrans(ctx, v.type, used_ids);
                    if (i < q.vars.size() - 1) {
                        str << ", ";
                    }
                }
                str << "] : (";
                // Add variables to context
                ctx.push_vars(q.vars);
                ppTrans(str, ctx, q.body, used_ids);
                str << ")";
                //Clear context
                ctx.pop_vars();
                return;
            }
            case Pred::PKind::Exists: {
                auto &q = p.toExists();
                str << "? [";
                for (size_t i = 0; i < q.vars.size(); ++i) {
                    const auto &v = q.vars.at(i);
                    str << localVarNameToString(v.name) << " : " << ppTrans(ctx, v.type, used_ids);
                    if (i < q.vars.size() - 1) {
                        str << ", ";
                    }
                }
                str << "] : (";
                ctx.push_vars(q.vars);
                ppTrans(str, ctx, q.body, used_ids);
                str << ")";
                ctx.pop_vars();
                return;
            }
        }
        assert(false); // unreachable
    }

    void
    add_local_defs(std::ostringstream &str, Context &env, const LocalEquations &local_eqs, const std::ostringstream &f,
                   std::set<std::string> &used_ids) {
        if (local_eqs.eqs.empty()) {
            str << f.str();
        } else {
            str << "! [";
            for (size_t i = 0; i < local_eqs.vars.size(); ++i) {
                auto &v = local_eqs.vars.at(i);
                str << localVarNameToString(v.name) << " : " << ppTrans(env, v.type, used_ids);
                if (i < local_eqs.vars.size() - 1)
                    str << ", ";
            }
            str << "] : (";
            if (local_eqs.eqs.size() == 1) {
                ppTrans_eq(str, env, local_eqs.eqs[0].first, local_eqs.eqs[0].second, used_ids);
            } else {
                str << "(";
                for (size_t i = 0; i < local_eqs.eqs.size(); ++i) {
                    auto &eq = local_eqs.eqs.at(i);
                    ppTrans_eq(str, env, eq.first, eq.second, used_ids);
                    if (i < local_eqs.eqs.size() - 1)
                        str << " & ";
                }
                str << ")";
            }
            str << " => (" << f.str() << "))";
        }
    }

    // Convert a set definition into a tptp string
    void ppTrans(std::ostringstream &str, Context &env, const pog::Set &set, std::set<std::string> &used_ids) {
        if (set.elts.empty()) {
            std::vector<Pred> conj;
            conj.push_back(Pred::makeNegation(Pred::makeExprComparison(
                    Pred::ComparisonOp::Equality,
                    Expr::makeIdent(set.setName.name, set.setName.type),
                    Expr::makeEmptySet(set.setName.type))));
            conj.push_back(Pred::makeExprComparison(
                    Pred::ComparisonOp::Membership,
                    Expr::makeIdent(set.setName.name, set.setName.type),
                    Expr::makeUnaryExpr(Expr::UnaryOp::Finite_Subsets, Expr::makeINTEGER(),
                                        BType::POW(BType::POW(BType::INT)))
            ));
            return ppTrans(str, env, Pred::makeConjunction(std::move(conj)), used_ids);
        } else {
            std::vector<Expr> elts;
            for (auto &elt: set.elts)
                elts.push_back(Expr::makeIdent(elt.name, elt.type));
            Pred eq = Pred::makeExprComparison(
                    Pred::ComparisonOp::Equality,
                    Expr::makeIdent(set.setName.name, set.setName.type),
                    Expr::makeNaryExpr(Expr::NaryOp::Set, std::move(elts), set.setName.type));
            ppTrans(str, env, eq, used_ids);
            str << " & ((";
            for (size_t i = 0; i < set.elts.size() - 1; i++) {
                auto current = set.elts[i];
                auto next = set.elts[i + 1];
                str << "(" << env.registerId(current.name, current.type, used_ids)
                    << " != " << env.registerId(next.name, next.type, used_ids) << ")";
                if (i < set.elts.size() - 2)
                    str << " & ";
            }
            str << "))";
            return;
        }
    }


    void printPrelude(
            std::ofstream &out,
            const std::string &minint,
            const std::string &maxint) {

    using std::string;

    const string prelude = R"(
tff(set_0_type,type,(set_0: $tType )).
tff(mem0_type,type,(mem0: ( $int * set_0 ) > $o )).
tff(set_1_type,type,(set_1: $tType )).
tff(mem1_type,type,(mem1: ( $real * set_1 ) > $o )).
tff(b_division_type,type,(divB: ( $int * $int ) > $int )).
tff(b_division_def_1,axiom,(! [X: $int,Y: $int] : (( $lesseq(0,X)& $less(0,Y))=> divB(X,Y) = $quotient_f(X,Y)))).
tff(b_division_def_2,axiom,(! [X: $int,Y: $int] : (( $lesseq(X,0)& $less(0,Y))=> divB(X,Y) = $uminus($quotient_f($uminus(X),Y)) ) )).
tff(b_division_def_3,axiom,(! [X: $int,Y: $int] : (( $lesseq(0,X)& $less(Y,0))=> divB(X,Y) = $quotient_f(X,Y)))).
tff(b_division_def_4,axiom,(! [X: $int,Y: $int] : (( $lesseq(X,0)& $less(Y,0))=> divB(X,Y) = $quotient_f($uminus(X),$uminus(Y)) ) )).
tff(exp_type,type,(exp: ( $int * $int ) > $int )).
tff(exp_def_1,axiom,(! [X: $int] : ( exp(X,0) = 1 ) )).
tff(exp_def_2,axiom,(! [X: $int,Y: $int] : (( $greatereq(Y,1))=> exp(X,Y) = $sum(X,exp(X,$difference(Y,1))) ) )).
tff(rexp_type,type,(rexp: ( $real * $int ) > $real )).
tff(rexp_def_1,axiom,(! [X: $real] : (( X != 0.0)=> rexp(X,0) = 1.0 ) )).
tff(rexp_def_2,axiom,(! [X: $real,Y: $int] : (( $greatereq(Y,1))=> rexp(X,Y) = $sum(X,rexp(X,$difference(Y,1))) ) )).
tff(isum_type,type,(isum: set_0 > $int )).
tff(isum_def_1,axiom,( ! [S: set_0] : (! [X: $int] : (~ mem0(X,S) ) => isum(S) = 0 ))).
tff(isum_def_2,axiom,( ! [S1: set_0,S2: set_0,E: $int,N: $int] : ( ( isum(S1) = N & ~ mem0(E,S1) & ! [X: $int] : ( mem0(X,S2) => ( X = E | mem0(X,S1) ) ) & ! [X: $int] :  ( mem0(X,S1) => mem0(X,S2) ) ) => isum(S2) = $sum(N,E) ) )).
tff(rsum_type,type,(rsum: set_1 > $real )).
tff(rsum_def_1,axiom,(! [S: set_1,X: $real] : ( ( ~ mem1(X,S))=> rsum(S) = 0.0 ) )).
tff(rsum_def_2,axiom,(! [S1: set_1, S2: set_1,E: $real,N: $real] : ( ( rsum(S1) = N & ~ mem1(E,S1) & ! [X: $real] : ( mem1(X,S2) => ( X = E | mem1(X,S1) ) ) & ! [X: $real] :  ( mem1(X,S1) => mem1(X,S2) ) ) => rsum(S2) = $sum(N,E) ) )).
tff(iprod_type,type,(iprod: set_0 > $int )).
tff(iprod_def_1,axiom,(! [S: set_0,X: $int] : ( ( ~ mem0(X,S))=> iprod(S) = 1 ) )).
tff(iprod_def_2,axiom,( ! [S1: set_0,S2: set_0,E: $int,N: $int] : ( ( iprod(S1) = N & ~ mem0(E,S1) & ! [X: $int] : ( mem0(X,S2) => ( X = E | mem0(X,S1) ) ) & ! [X: $int] : ( ( X = E | mem0(X,S1) ) => mem0(X,S2) ) ) => isum(S2) = $product(N,E) ) )).
tff(rprod_type,type,(rprod: set_1 > $real )).
tff(rprod_def_1,axiom,(! [S: set_1,X: $real] : ( ( ~ mem1(X,S))=> rprod(S) = 1.0 ) )).
tff(rprod_def_2,axiom,(! [S1: set_1,S2: set_1,E: $real,N: $real] : ( ( rprod(S1) = N & ~ mem1(E,S1) & ! [X: $real] : ( mem1(X,S2) => ( X = E | mem1(X,S1) ) ) & ! [X: $real] : ( ( X = E | mem1(X,S1) ) => mem1(X,S2) ) ) => rprod(S2) = $sum(N,E) ) )).
tff(min_int_type,type,(min_int: $int )).
tff(max_int_type,type,(max_int: $int )).
        )";
    const string tff_min_int_axiom = "tff(min_int_axiom,axiom,(min_int = $uminus(" + minint + " ))).";
    const string tff_max_int_axiom = "tff(max_int_axiom,axiom,(max_int =" + maxint+ ")).";

    out << "%--------------------------------------------------------------------------" << endl
        << "% File     : ." << endl
        << "% Domain   : Atelier-B Proof Obligations - translated to TPTP" << endl
        << "% Problem  : PUZ005+1" << endl
        << "% Version  : TPTP v5.3.0. Released v2.2.0." << endl
        << "% English  : " << endl
        << "% Comments : Autogenerated by PPTRANS-TPTP" << endl
        << "% Source   : " << endl
        << "%--------------------------------------------------------------------------" << endl
        << prelude << endl
        << tff_min_int_axiom << endl
        << tff_max_int_axiom << endl
        << "%--------------------------------------------------------------------------" << endl;
}

} // namespace ppTransTPTP
