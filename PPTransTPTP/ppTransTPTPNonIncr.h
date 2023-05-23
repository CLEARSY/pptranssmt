/** ppTransNonIncr.h

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
#ifndef PPTRANSTPTPNONINCR_H
#define PPTRANSTPTPNONINCR_H

#include<map>
#include<set>
#include<string>
#include<vector>

#include "pog.h"

namespace ppTransNonIncr {
using std::map;
using std::pair;
using std::set;
using std::string;

/**
     * @brief saveTPTPFileNonIncr produces a TPTP
     * encoding one goal.
     * @param pog the POG DOM containing the goal to be encoded
     * @param filename path to file where TPTP script is saved
     * @param groupIdx position of the PO group containing the goal to be encoded in the DOM
     * @param goalIdx position of the goal in the PO group in the DOM
     * @param rp number of rp steps to apply
     * @param dd indicates if dd command applies
     * @param model indicates if a model has to be queried
     * @param minint
     * @param maxint
     */
extern void saveTPTPFileNonIncrOne(
        pog::Pog &pog,
        const string &filename,
        // PO
        int groupIdx, int goalIdx,
        // Other
        int rp,
        bool dd,
        bool model,
        const string &minint = "2147483648",
        const string &maxint = "2147483647");

/**
     * @brief saveTPTPFileNonIncr saves the translation to non-incremental TPTP
     * (one file per goal)
     * @param pog
     * @param path
     * @param rp
     * @param dd
     * @param model
     * @param minint
     * @param maxint
     */
    extern void saveTPTPFileNonIncr(
            pog::Pog &pog,
            const string &path,
            int rp,
            bool dd,
            bool model,
            const string &minint = "2147483648",
            const string &maxint = "2147483647");
}

#endif // PPTRANSTPTPNONINCR_H
