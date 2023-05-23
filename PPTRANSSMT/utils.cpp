/** utils.cpp

   \copyright Copyright © CLEARSY 2022
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

#include<filesystem>
#include<string>

#include "utils.h"

namespace utils {

using std::string;

string absoluteBasename(const string& filename) {
    // The next lines build the prefix of the paths common to all files that will
    // be output: "absolutePath/baseName"
    // The result is stored in variable `prefix`.
    // Note: The path class provided in C++17 contains all the functionalities to implement the following lines.
    // get the path up but the file extension
    const std::filesystem::path thePath {filename};
    const std::filesystem::path result {thePath.parent_path() / thePath.stem()};
    return result.string();
}

string absoluteFilePath(const string& filename) {
    const std::filesystem::path thePath {filename};
    const std::filesystem::path result {thePath.parent_path() / thePath.filename()};
    return result.string();
}

}
