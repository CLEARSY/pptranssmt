
# This script is used to compare the translation of an POG obligation from Atelier B to either SMT or TPTP
# Usage: translate.sh <proof-obligation-directory> 

# The script will create a the output directory if it does not exist.
# The script will create a file in the output directory with the same name as the proof obligation

# The script will return 0 if the translation was successful, and 1 otherwise
# Check that the correct number of arguments were provided
if [ $# -ne 1 ]; then
    echo "Usage: translate.sh <proof-obligation-directory>"
    exit 1
fi


# Check that the proof obligation directory exists
if [ ! -d $1 ]; then
    echo "Proof obligation directory $1 does not exist"
    exit 1
fi

# Create a directory for the output files
output_dir_base=$(pwd)/tptp_smt_comparison

# Check that the output directory exists and create it if it does not
if [ ! -d $output_dir_base ]; then
    mkdir $output_dir_base
fi

# Build the translators using Cmake and make
cmake .
make

# Ensure that vampire is in the path
if ! command -v vampire_z3 &> /dev/null
then
    echo "vampire could not be found - please ensure that vampire is in the path"
    exit 1
fi

# Ensure that z3 is in the path
if ! command -v z3 &> /dev/null
then
    echo "z3 could not be found - please ensure that z3 is in the path"
    exit 1
fi


# Translate the proof-obligation files
for file in $1/**.pog; do
    # Get the filename and extension of the proof obligation
    filename=$(basename -- $file)
    extension="${filename##*.}"
    filename="${filename%.*}"

    # Get current directory
    output_dir=$output_dir_base/$filename

    # Check that the output directory exists and create it if it does not
    if [ ! -d $output_dir ]; then
        mkdir $output_dir
    fi

    # Translate the file
    echo "translating $file to $output_dir"

    # Write header to log file
    echo "File, Proved by Z3, Proved by Vampire" > $output_dir/results_combined.csv

    ./PPTransTPTP/PPTransTPTP -i $file -o $output_dir/$filename
    if [ $? -ne 0 ]; then
        echo "TPTP translation failed"
        exit 1
    fi

    ./PPTransSMT/PPTransSMT -i $file -o $output_dir/$filename -n
    if [ $? -ne 0 ]; then
        echo "SMT translation failed"
        exit 1
    fi


    # Run z3 on the translated files
    for f in $output_dir/**.smt2; do
        # Get the filename without the extension
        filename_no=$(basename -- $f)
        filename_no=${filename_no%.*}
        z3_success=0
        vampire_success=0
        z3 -smt2 smt.ematching=false -T:5 $f > $output_dir/${filename_no}_smt.out
        # Check that z3 was successful by checking whether the output contains the string "unsat"
        if grep -q "unsat" $output_dir/${filename_no}_smt.out; then
            echo "Z3 proved $f"
            z3_success=1
        else
            echo "Warning: Z3 failed to prove $f"
        fi

        tptp_file=$output_dir/${filename_no}.tptp
        # Run vampire on the translated files
        vampire_z3 --mode casc --input_syntax tptp --time_limit 2 $tptp_file > $output_dir/${filename_no}_tptp.out
                # Check that vampire was successful by checking whether the output contains the string "Refutation found"
        if grep -q "Refutation found" $output_dir/${filename_no}_tptp.out; then
            echo "Vampire proved $tptp_file"
            vampire_success=1
        else
            echo "Warning: Vampire failed to prove $tptp_file"
        fi
        # Write the results to the results file
        echo "$filename_no, $z3_success, $vampire_success" >> $output_dir/results_combined.csv
    done
done