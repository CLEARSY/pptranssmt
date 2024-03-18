# This script is used to translate an POG obligation to either SMT or TPTP

# Usage: translate.sh <proof-obligation> <TPTP2X path>

# The script will create a the output directory if it does not exist.
# The script will create a file in the output directory with the same name as the proof obligation

# The script will return 0 if the translation was successful, and 1 otherwise
# Check that the correct number of arguments were provided
if [ $# -ne 2 ]; then
    echo "Usage: translate.sh <proof-obligation>  <TPTP2X path>"
    exit 1
fi


# Check that the proof obligation exists
if [ ! -f $1 ]; then
    echo "Proof obligation $1 does not exist"
    exit 1
fi

# Get the filename and extension of the proof obligation
filename=$(basename -- $1)
extension="${filename##*.}"
filename="${filename%.*}"

# Get current directory
output_dir_base=$(pwd)/tptp_generated

# Check that the output directory exists and create it if it does not
if [ ! -d $output_dir_base ]; then
    mkdir $output_dir_base
fi

output_dir=$output_dir_base/$filename
if [ ! -d $output_dir ]; then
    mkdir $output_dir
fi

# Build the translator using Cmake and make
cmake .
make

# Translate the file
echo "translating $1 to $output_dir"

./PPTransTPTP/PPTransTPTP -i $1 -o $output_dir/$filename
if [ $? -ne 0 ]; then
    echo "Translation failed"
    exit 1
fi

# For TPTP, we need to format the output file using the TPTP2X utility
# Check that the TPTP2X utility exists
if [ ! -f $q ]; then
    echo "TPTP2X utility does not exist at $2"
    exit 1
fi

# Check that the TPTP2X utility is executable
if [ ! -x $2 ]; then
    echo "TPTP2X utility $2 is not executable"
    exit 1
fi

# Make an output directory for formatted files
mkdir $output_dir/formatted

# Run the TPTP2X utility on all files in the output directory
for file in $output_dir/*.tptp; do
    echo "Formatting $file"
    $2 -d $output_dir/formatted/ $file
    # Delete the unformatted file
    rm $file
done

# Move the formatted files to the output directory
mv $output_dir/formatted/* $output_dir

# Delete the formatted directory
rm -r $output_dir/formatted

# Ensure that vampire is in the path
if ! command -v vampire_z3 &> /dev/null
then
    echo "vampire could not be found"
    exit 1
fi

# Write header to log file
echo "File, Proved by Vampire (1 means prover/ 0 means not proved)" > $output_dir/results.csv

# Run vampire on the translated file
for file in $output_dir/**.tptp; do
    # Get the filename without the extension
    filename_no=$(basename -- $file)
    filename_no=${filename_no%.*}
    vampire_z3 --mode casc --input_syntax tptp -p proofcheck --time_limit 3 $file > $output_dir/$filename_no.out
    # Check that vampire was successful by checking whether the output contains the string "Refutation found"
    if grep -q "Refutation found" $output_dir/$filename_no.out; then
        echo "Vampire proved $file"
        echo "$file, 1" >> $output_dir/results.csv
    else
        echo "Warning: Vampire failed to prove $file"
        echo "$file, 0" >> $output_dir/results.csv
    fi
done
