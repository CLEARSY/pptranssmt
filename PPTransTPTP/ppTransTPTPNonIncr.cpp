/** ppTransNonIncr.cpp

   \copyright Copyright © CLEARSY 2023
   \license This file is part of ppTransTPTP.

   ppTransTPTP is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   ppTransTPTP is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with ppTransTPTP. If not, see <https://www.gnu.org/licenses/>.
*/
#include "ppTransTPTPNonIncr.h"
#include "ppTransTPTP.h"
#include "decomposition.h"
#include "utils.h"

#include <fstream>
#include <iostream>

using namespace std;

namespace ppTransNonIncr {

    static void merge(std::set<std::string> &set, const std::set<std::string> &to_insert) {
        for (auto &e: to_insert)
            set.insert(e);
    }

    static void merge(std::set<VarName> &set, const std::set<VarName> &to_insert) {
        for (auto &e: to_insert)
            set.insert(e);
    }

    static bool is_intersecting(
            const std::set<VarName> &set1,
            const std::set<VarName> &set2) {
        if (set1.empty())
            return false;

        for (auto &v: set2) {
            if (set1.find(v) != set1.end())
                return true;
        }
        return false;
    }

    static bool keepHyp(
            const std::set<VarName> &rpVars,
            const Pred &p) {
        if (rpVars.empty())
            return false;
        std::set<VarName> fv_p;
        p.getFreeVars({}, fv_p);
        return is_intersecting(rpVars, fv_p);
    }

    static bool keepHyp(
            const std::set<VarName> &rpVars,
            const pog::Set &set) {
        if (rpVars.empty())
            return false;
        std::set<VarName> fv_set;
        fv_set.insert(set.setName.name);
        for (auto &e: set.elts)
            fv_set.insert(e.name);
        return is_intersecting(rpVars, fv_set);
    }

    static int findDef(
            const std::vector<pog::Define> &vec,
            const std::string &d) {
        int pos = -1;
        for (size_t i = 0; i < vec.size(); i++) {
            if (vec[i].name == d) {
                pos = i;
                break;
            }
        }
        assert(pos >= 0); // exception
        return pos;
    }

    using translation_t = std::pair<std::string, std::set<std::string>>;

    static void updateRpVars(
            std::set<VarName> &rpVars,
            const Pred &h) {
        if (rpVars.empty())
            return;
        std::set<VarName> fv_h;
        h.getFreeVars({}, fv_h);
        if (is_intersecting(rpVars, fv_h))
            merge(rpVars, fv_h);
    }

    static void updateRpVars(
            std::set<VarName> &rpVars,
            const pog::Set &set) {
        if (rpVars.empty())
            return;
        std::set<VarName> fv_set;
        fv_set.insert(set.setName.name);
        for (auto &e: set.elts)
            fv_set.insert(e.name);
        if (is_intersecting(rpVars, fv_set))
            merge(rpVars, fv_set);
    }

    void saveTPTPFileNonIncr(
            const pog::Pog &pog,
            ppTransTPTP::Context &env,
            // Cache
            std::map<std::pair<std::string, int>, translation_t> &definitionHyps_tr,
            std::map<std::pair<std::string, int>, translation_t> &definitionSets_tr,
            std::map<int, translation_t> &globalHyps_tr,
            std::map<int, translation_t> &localHyps_tr,
            // PO
            int group_nb, int goal_nb,
            // Other
            int rp,
            bool dd,
            bool model,
            const std::string &filename,
            const std::string &minint,
            const std::string &maxint) {
        const pog::POGroup &group = pog.pos[group_nb];
        const pog::PO &sg = pog.pos[group_nb].simpleGoals[goal_nb];

        // Computing rpVars
        std::set<VarName> rpVars;
        if (rp >= 1) {
            sg.goal.getFreeVars({}, rpVars);
            if (!dd) {
                for (auto &ref: sg.localHypsRef) {
                    group.localHyps[ref - 1].getFreeVars({}, rpVars);
                }
            }
            for (int lrp = rp - 1; lrp > 0; lrp--) {
                // defs
                for (auto &s: group.definitions) {
                    const pog::Define &def = pog.defines[findDef(pog.defines, s)];
                    for (auto &e: def.contents) {
                        if (std::holds_alternative<Pred>(e)) {
                            const auto &hyp {std::get<Pred>(e)};
                            updateRpVars(rpVars, hyp);
                        } else {
                            const auto &set {std::get<pog::Set>(e)};
                            updateRpVars(rpVars, set);
                        }
                    }
                }
                // global hyps
                for (auto &h: group.hyps)
                    updateRpVars(rpVars, h);
                // local hyps
                if (dd) {
                    for (auto &ref: sg.localHypsRef)
                        updateRpVars(rpVars, group.localHyps[ref - 1]);
                }
            }
        }

        std::set<std::string> used_ids;

        // definitions
        std::map<std::string, std::string> defines_tr; // name -> translation
        for (auto &def_name: group.definitions) {
            const pog::Define &def = pog.defines[findDef(pog.defines, def_name)];
            const std::string &s1 {def.name};
            for (size_t j = 0; j < def.contents.size(); ++j) {
                const auto &s2 {def.contents[j]};
                string tr;
                if (std::holds_alternative<Pred>(s2)) {
                    const auto &hyp {std::get<Pred>(s2)};
                    if (rp < 0 || keepHyp(rpVars, hyp)) {
                        auto it = definitionHyps_tr.find({def_name, j});
                        if (it != definitionHyps_tr.end()) {
                            merge(used_ids, it->second.second);
                            tr = it->second.first;
                        } else {
                            std::set<std::string> used_ids2;
                            std::ostringstream str;
                            ppTransTPTP::ppTrans(str, env, hyp, used_ids2);
                            definitionHyps_tr[{def_name, j}] = {str.str(), used_ids2};
                            merge(used_ids, used_ids2);
                            tr = str.str();
                        }
                    }
                } else {
                    const auto &set {std::get<pog::Set>(s2)};
                    if (rp < 0 || keepHyp(rpVars, set)) {
                        auto it = definitionSets_tr.find({def_name, j});
                        if (it != definitionSets_tr.end()) {
                            merge(used_ids, it->second.second);
                            tr = it->second.first;
                        } else {
                            std::set<std::string> used_ids2;
                            std::ostringstream str;
                            ppTransTPTP::ppTrans(str, env, set, used_ids2);
                            definitionSets_tr[{def_name, j}] = {str.str(), used_ids2};
                            merge(used_ids, used_ids2);
                            tr = str.str();
                        }
                    }
                }
                const std::string label {s1 + ":" + std::to_string(j)};
                defines_tr[label] = tr;
            }
        }

        // Global hyps
        std::vector<std::string> globalHyps;
        for (size_t ref = 0; ref < group.hyps.size(); ref++) {
            const Pred &hyp = group.hyps[ref];
            if (rp < 0 || keepHyp(rpVars, hyp)) {
                auto it = globalHyps_tr.find(ref);
                if (it != globalHyps_tr.end()) {
                    merge(used_ids, it->second.second);
                    globalHyps.push_back(it->second.first);
                } else {
                    std::set<std::string> used_ids2;
                    std::ostringstream str;
                    ppTransTPTP::ppTrans(str, env, hyp, used_ids2);
                    globalHyps_tr[ref] = {str.str(), used_ids2};
                    merge(used_ids, used_ids2);
                    globalHyps.push_back(str.str());
                }
            }
        }

        // Local Hyps
        std::vector<std::pair<int, std::string>> localHyps; // ref-1 -> translation
        for (auto &ref: sg.localHypsRef) {
            if (rp < 0 || !dd || keepHyp(rpVars, group.localHyps[ref - 1])) {
                auto it = localHyps_tr.find(ref);
                if (it != localHyps_tr.end()) {
                    merge(used_ids, it->second.second);
                    localHyps.push_back(std::make_pair(ref-1, it->second.first));
                } else {
                    std::set<std::string> used_ids2;
                    std::ostringstream str;
                    ppTransTPTP::ppTrans(str, env, group.localHyps[ref - 1], used_ids2);
                    localHyps_tr[ref] = {str.str(), used_ids2};
                    merge(used_ids, used_ids2);
                    localHyps.push_back(std::make_pair(ref-1, str.str()));
                }
            }
        }

        // Goal
        std::ostringstream goal;
        ppTransTPTP::ppTrans(goal, env, sg.goal, used_ids);

        // Printing
        std::ofstream out;
        out.open(filename);
        out << "% PO " << group_nb << " " << goal_nb << endl;
        out << "% Group " << group.tag << endl;
        out << "% Tag " << sg.tag << endl;
        ppTransTPTP::printPrelude(out, minint, maxint);
        out << "% Global declarations" << endl;
        for (auto &s: used_ids) {
            if (s != "mem0" && s != "mem1" && s != "set_0" && s != "set_1")
                out << env.getTPTPDeclarations().find(s)->second << endl;
        }
        out << "% Defines" << endl;
        int counter = 0;
        for (auto &d: defines_tr) {
            out << "tff('Define:" << d.first << "', axiom, " << d.second << ")." << endl << endl;
        }
        out << "%Global hypotheses" << endl;
        counter = 0;
        for (auto &hyp: globalHyps)
            out << "tff('gh_" << counter++ << "_def'" << ", hypothesis, " << hyp << ")." << endl << endl;
        out << "% Local hypotheses" << endl;
        counter = 0;
        for (auto &hyp: localHyps)
            out << "tff('Local_Hyp:" << hyp.first << "'" << ", hypothesis, " << hyp.second << ")." << endl << endl;
        out << "% Goal" << endl;
        out << "tff('Goal', conjecture, " << goal.str() << ")." << endl;
        out.close();
    }

    void saveTPTPFileNonIncrOne(
            pog::Pog &pog,
            const string &filename,
            // PO
            int groupIdx,
            int goalIdx,
            // Other
            int rp,
            bool dd,
            bool model,
            const string &minint,
            const string &maxint) {
        std::map<std::pair<std::string, int>, translation_t> definitionHyps_tr;
        std::map<std::pair<std::string, int>, translation_t> definitionSets_tr;
        std::map<int, translation_t> globalHyps_tr;
        std::map<int, translation_t> localHyps_tr;
        ppTransTPTP::Context env;
        decomp::decompose(pog);
        std::string absolutefilename{utils::absoluteFilePath(filename)};
        saveTPTPFileNonIncr(pog, env, definitionHyps_tr, definitionSets_tr,
                            globalHyps_tr, localHyps_tr, groupIdx, goalIdx, rp, dd, model,
                            absolutefilename, minint, maxint);
    }

    void saveTPTPFileNonIncr(
            pog::Pog &pog,
            const std::string &filename,
            int rp,
            bool dd,
            bool model,
            const std::string &minint,
            const std::string &maxint) {
        using std::to_string;
        std::string prefix{utils::absoluteBasename(filename)};

        std::map<std::pair<std::string, int>, translation_t> definitionHyps_tr;
        std::map<std::pair<std::string, int>, translation_t> definitionSets_tr;
        ppTransTPTP::Context env;
        decomp::decompose(pog);
        for (size_t group_nb = 0; group_nb < pog.pos.size(); group_nb++) {
            std::map<int, translation_t> globalHyps_tr;
            std::map<int, translation_t> localHyps_tr;
            for (size_t po_nb = 0; po_nb < pog.pos[group_nb].simpleGoals.size(); po_nb++) {
                std::string path{prefix + "-" + to_string(group_nb) + "-" + to_string(po_nb) + ".tptp"};
                saveTPTPFileNonIncr(pog, env, definitionHyps_tr, definitionSets_tr,
                                    globalHyps_tr, localHyps_tr, group_nb, po_nb, rp, dd, model,
                                    path, minint, maxint);
            }
        }
    }
}
