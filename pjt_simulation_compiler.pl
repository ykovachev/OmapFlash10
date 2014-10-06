# The purpose of this file is:
#   1) ensure that target is rebuild if project file is changed
#   

use strict;
my $in = $ARGV[0];
my $out = "$in.c";
open IN, $in or die "$in $!";
open OUT, ">$out" or die "$out $!";

print "$in\n";
print OUT "/* DO NOT EDIT generated file from $in by $0 */";
while(<IN>){
}

