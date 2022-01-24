#!/usr/bin/python3
################################################################################
#
# tests.py
#
#   Run each test specified in a test file and report success/fail.
#
# test file format
#
#   Each line in the file describes an XML to run and the validation outputs.
#   The XML should call an instance of the 'ValidationStrategy' and indicate all
#   of the expected output files with 'GoldStandard' nodes. This strategy only
#   writes outputs files that fail to match the gold standard. Any outputs which
#   are written indicate a failed test.
#
################################################################################

# Imports

import sys, os, subprocess

# Configuration
xml_dir = 'XMLs/'
output_dir = 'Scratch/'

# Parse arguments
if len(sys.argv) != 2:
  print("usage: tests.py <test file>")
  sys.exit(1)
test_file = sys.argv[1]

# Assert the test file exists.
if not os.path.isfile(test_file):
  print('Error: cannot find test file ' + test_file + '.')
  sys.exit(1)

# Delete any leftover files from the previous run.
for root, dirs, files in os.walk(output_dir):
  for f in files:
    # Skip hidden files.
    if f[0] == '.':
      continue;
    os.remove(os.path.join(root, f))

# Read the test file into an array of lines.
lines = [line.strip() for line in open(test_file, 'r')]

# Run each test.
for line in lines:
  # Skip empty lines.
  if not line:
    continue

  # Split the line into tokens.
  tokens = line.split()
  if not tokens or len(tokens) < 2:
    print('Error: ill-formed test on line', lines.index(line) + 1, ': ', line)
    sys.exit(1)
  xml_file, *output_files = tokens

  # Execute the XML file.
  xml_path = xml_dir + xml_file
  debug_output = output_dir + os.path.basename(xml_file) + '.pmpl'
  result = subprocess.run('../pmpl -f ' + xml_path + ' >& ' + debug_output,
      shell=True)

  # Check for aborted run.
  if result.returncode != 0:
    print('Error: test', xml_file, 'failed to run.')
    continue

  # Check for errors (i.e. an output file was generated).
  for output_file in output_files:
    output_path = output_dir + output_file
    # No file = no problem.
    if not os.path.isfile(output_path):
      print('Success: test', xml_file, 'on output', output_file)
      continue

    # Rut roh.
    print('Error: test', xml_file, 'failed to match correct output on',
        output_file)
