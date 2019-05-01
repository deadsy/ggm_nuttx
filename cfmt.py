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
  'nbad',
  'bap',
  'bbb',
  'nbbo',
  'nbc',
  'bl',
  'bl2',
  'bls',
  'nbs',
  'cbi2',
  'ncdw',
  'nce',
  'ci2',
  'cli0',
  'cp40',
  'ncs',
  'nbfda',
  'nbfde',
  'di1',
  'nfc1',
  'fca',
  'i2',
  'l80',
  'lp',
  'ppi2',
  'lps',
  'npcs',
  'pmt',
  'nprs',
  'npsl',
  'saf',
  'sai',
  'sbi2',
  'saw',
  'sc',
  'sob',
  'nss',
  'nut',
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
	('files/nuttx', fmt_nuttx),
	('files/apps/ggm', fmt_linux),
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
