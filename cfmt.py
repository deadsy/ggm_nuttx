#!/usr/bin/env python3
# Format project source code.

#------------------------------------------------------------------------------

import glob
import subprocess
import os
import fnmatch
import sys

#------------------------------------------------------------------------------

indent_exec = '/usr/bin/indent'
uncrustify_exec = '/usr/local/bin/uncrustify'
nxstyle_exec = '/usr/local/bin/nxstyle'

#------------------------------------------------------------------------------

class srcfile(object):

  def __init__(self, name, formatter):
    self.name = name
    self.formatter = formatter

  def dfilter(self, dlist):
    """return true if the file is in a filtered directory"""
    for d in dlist:
      if self.name.startswith(d):
        return True
    return False

  def ffilter(self, flist):
    """return true if the file is filtered"""
    return self.name in flist

  def format(self):
    """format the file"""
    return self.formatter(self.name)

#------------------------------------------------------------------------------

def exec_cmd(cmd):
  """execute a command, return the output and return code"""
  output = ''
  rc = 0
  try:
    output = subprocess.check_output(cmd, shell=True)
  except subprocess.CalledProcessError as x:
    rc = x.returncode
  return output, rc

#------------------------------------------------------------------------------

def exec_indent(fname, opts):
  exec_cmd('%s %s %s' % (indent_exec, opts, fname))
  os.unlink('%s~' % fname)

#------------------------------------------------------------------------------

def fmt_linux(fname):
  print("fmt_linux %s" % fname)
  opts = '-brf -linux -l10000'
  exec_indent(fname, opts)

#------------------------------------------------------------------------------

nuttx_indent_opts = (
  'l80', # Set maximum line length for non-comment lines to n.
  'lc200', # Set maximum line length for comment formatting to n.
  'nut', # Use spaces instead of tabs.
  'i2', # Set indentation level to n spaces.
  'saf', # Put a space after each for.
  'sai', # Put a space after each if.
  'sob', # Swallow optional blank lines.
  'nbs', # Do not put a space between sizeof and its argument.
  'bli2', # Indent braces n spaces.
  'nprs', # Do not put a space after every ’(’ and before every ’)’.
  'npsl',# Put the type of a procedure on the same line as its name.
  'ncs', # no-space-after-casts
  'npcs', # Do not put space after the function in function calls.
  'pmt', # Preserve access and modification times on output files.
  'lps', # Leave space between ‘#’ and preprocessor directive.
)

nuttx_indent_opts_old = (
  'nbad', # Do not force blank lines after declarations.
  'bap', # Force blank lines after procedure bodies.
  'bbb', # Force blank lines before block comments.
  'nbbo', # Do not prefer to break long lines before boolean operators.
  'nbc', # Do not force newlines after commas in declarations.
  'bl', # Put braces on line after if, etc.
  'bl2', # ??
  'bli2', # Indent braces n spaces.
  'bls', # Put braces on the line after struct declaration lines.
  'nbs', # Do not put a space between sizeof and its argument.
  'cbi2', # Indent braces after a case label N spaces.
  'ncdw', # Do not cuddle } and the while of a do {} while;
  'nce', # Do not cuddle } and else.
  'ci2', # Continuation indent of n spaces.
  'cli0', # Case label indent of n spaces.
  'cp40', # Put comments to the right of #else and #endif statements in column n.
  'ncs', # no-space-after-casts
  'nbfda', # dont-break-function-decl-args
  'nbfde', # dont-break-function-decl-args-end
  'di1', # Put variables in column n.
  'nfc1', # Do not format comments in the first column as normal.
  'fca', # Do not disable all formatting of comments.
  'i2', # Set indentation level to n spaces.
  'l80', # Set maximum line length for non-comment lines to n.
  'lc80', # Set maximum line length for comment formatting to n.
  'lp', # Line up continued lines at parentheses.
  'ppi2',# Specify the indentation for preprocessor conditional statements.
  'lps', # Leave space between ‘#’ and preprocessor directive.
  'npcs', # Do not put space after the function in function calls.
  'pmt', # Preserve access and modification times on output files.
  'nprs', # Do not put a space after every ’(’ and before every ’)’.
  'npsl',# Put the type of a procedure on the same line as its name.
  'saf', # Put a space after each for.
  'sai', # Put a space after each if.
  'sbi2',# Indent braces of a struct, union or enum N spaces.
  'saw', # Put a space after each while.
  'sc',  # Put the ‘*’ character at the left of comments.
  'sob', # Swallow optional blank lines.
  'nss', # Do not force a space before the semicolon after certain statements.
  'nut', # Use spaces instead of tabs.
)

def fmt_nuttx(fname):
  print("fmt_nuttx on %s" % fname)
  opts = ' '.join(["-%s" % opt for opt in nuttx_indent_opts])
  exec_indent(fname, opts)
  #exec_cmd('%s -c ./tools/uncrustify.cfg -q --no-backup %s' % (uncrustify_exec, fname))
  #exec_cmd('%s -m 90 %s' % (nxstyle_exec, fname))

#------------------------------------------------------------------------------

def get_files(dlist, fo_flist, fo_dlist):
  # get the full file list
  flist = []
  for d, formatter in dlist:
    for root, dname, fnames in os.walk(d):
      for fname in fnmatch.filter(fnames, '*.[ch]'):
        flist.append(srcfile(os.path.join(root, fname), formatter))
  # run the filters
  flist = [f for f in flist if not f.dfilter(fo_dlist)]
  flist = [f for f in flist if not f.ffilter(fo_flist)]
  return flist

#------------------------------------------------------------------------------

# *.c and *.h files in these directories will be auto-formatted.
src_dirs = (
	('nuttx/configs/axoloti', fmt_nuttx),
	('nuttx/configs/imxrt1020-evk', fmt_nuttx),
)

# don't format directories in this list
filter_dirs = (
)

# don't format files in this list
filter_files = (
)

#------------------------------------------------------------------------------

def main():
  if len(sys.argv) == 2:
    f = srcfile(sys.argv[1], fmt_nuttx)
    f.format()
  else:
    flist = get_files(src_dirs, filter_files, filter_dirs)
    for f in flist:
      f.format()
  sys.exit(0)

main()

#------------------------------------------------------------------------------
