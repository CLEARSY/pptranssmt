# ppTransSmt / ppTransTPTP

Tool translating proof obligations from the POG format
to SMT-LIB 2.6 based on the pptrans approach.
or to TPTP format based on the pptrans approach.

* BAST: a C++ library to represent B Abstract Syntax Trees. Depends on Qt5Core, Qt5Xml.

* POGLoader: a C++ library to load POG files to memory, using the BAST
  representation. Depends on BAST, Qt5Core, Qt5Xml.

* PPTRANSSMT: a C++ program to translate POG files to SMTLIB-2.6
  format, based on an extension of the pptrans encoding described in the paper [Integrating SMT solvers in Rodin](http://dx.doi.org/10.3233/SAT190123).

* PPTRANSSMT: a C++ program to translate POG files to TPTP (tff)
  format, in a similar way to the SMTLIB-2.6 translation.

## COMPILING

The code uses Qt libraries. It is known to compile with Qt5.12.x and Qt5.15.x under
Linux.

Our build process is based on `cmake`, which produces suitable Makefiles from the `CMakeLists.txt` provided here. To build the code, run the following commands
```
cmake .
make
```

To update the repository and its submodules, use the following command:

```bash
git submodule update --init --recursive
```

If this repository is embedded in a build process managed by CMake, then
that process is responsible for processing the CMakeLists.txt file of the
BAST library before that of this project.

## USAGE

### Using ppTransTPTP

```bash
  ./ppTransTPTP -i <input.pog> -o <output-tptp-folder>
```

### Scripts

Alternatively, you can use one of the following scripts in the `Scripts` folder:

* `translate.sh` : wrapper around `pptranstptp` that takes as input a POG file and outputs a TPTP file.  

* `comparison.sh` takes as input a folder of POG files and uses both `pptranstptp` and `pptranssmt` to generate TPTP and SMTLIB files respectively. It then uses the `vampire` and `z3` solvers to check the validity of the generated files. The results are saved in a CSV file. The script requires the `vampire` and `z3` solvers to be installed and available in the `PATH` environment variable.

Both scripts will compile the code (using cmake and make) if needed.

## COPYING

This software is copyright (C) CLEARSY 2023. All rights reserved.

The source code is distributed under the terms of the GNU General Public Licence (GNU GPL) Version 3.
