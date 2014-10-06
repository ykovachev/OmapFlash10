#copy to C:\CCSTG_C\a0394914_omapflash2s_view\OMAPSW_DP\omapflash\3430\build
#run from C:\CCSTG_C\a0394914_omapflash2s_view\OMAPSW_DP\omapflash\3430\build
# output %1.cmm run this from lauterbach to get symbols
# output sorted list of public symbols sizes (before optimizing the biggest functions for size go look at assember to check that it is not followed by a static function as static functions do not show up in map file)

use strict;

#GLOBAL SYMBOLS: SORTED BY Symbol Address 
#
#address    name
#--------   ----
#00000000   __CODE_START
#0000001c   VEC_TABLE_SIZE
#00000200   EXCEPTION_STACK_SIZE
#00000682   _CONST_SECTION_SIZE
#00000800   EXCEPTION_STACK_TOTAL
#00001000   __STACK_SIZE
#001a0000   __SYSMEM_SIZE
#40200000   _reset
#4020007c   _HEAP_SIZE
#40200080   _CODE_START

#my $dir=q(C:\CCSTG_C\a0394914_omapflash2s_view\OMAPSW_DP\omapflash\_out_\bin\3430\Debug\dnld_startup_3430sdp_gp_romapi.);
my $dir=q(C:\CCSTG_C\a0394914_omapflash3.1s_view\OMAPSW_DP\omapflash\_out_\bin\\);
#my $dir=q(M:\a0394914_omapflash_2_view\OMAPSW_DP\omapflash\_out_\bin\3430\Debug\dnld_startup_3430sdp_gp_romapi.);
#my $dir=q(M:\a0394914_omapflash_2_view\OMAPSW_DP\omapflash\_out_\bin\\);
my %objs;
my @objs;

sub read_map{
    my ($file) = @_;
    $file =~ s/\.map$//;
    my $in = "$file.map";
    my $cmm = "$file.cmm";
    my $out = "$file.map.out";
    open IN, $in || die $!;
    open CMM, ">$cmm" || die "$! $cmm";
    open OUT, ">$out" || die "$! $out";
    
    my $armed;
    my $offset;
    my %lines;
    my $line;
    my $lib;
    while (<IN>){
        #        1 section    2 page    3 origin        4 size          56 lib             7 obj 
        #if (/^(?:(\.\w+|\*)\s+(\d+))?\s+([\da-f]{8}) {4}([\da-f]{8}) {5}((\w+\.lib)?\s+:)? (\w\.obj) /) {
        if (/^(?:(\.\w+|\*)\s+(\d+))?\s+([\da-f]{8}) {4}([\da-f]{8}) {5}((\w+\.lib)?\s+: )?(\w+\.obj .*?)\s*$/) {
            if ($5) {
                if ($6) {
                    $lib = $6;
                }
            }
            else {
                $lib = "";
            }
            $objs{$7} = {
                origin=>$3,
                size=>$4,
                name=>$7,
                lib=>$lib,
            };
            push @objs, $objs{$7};
        }
        if (/^([\da-f]{8})\s/) {
            if (exists $objs[-1]{name}){
                push @objs, {origin=>sprintf("%08x",eval("0x$objs[-1]{origin}+0x$objs[-1]{size}"))};
            }
            for (my $i = 0; $i < @objs; $i++){
                if ($objs[$i]{origin} le $1){
                    if ($i + 1 < @objs && $objs[$i+1]{origin} gt $1){
                        my $obj = $objs[$i];
                        s/\s*$//;
                        $_ = sprintf "%-40s %-20s $obj->{name}\n", $_, $obj->{lib};
                        last;
                    }
                }
                elsif ($objs[$i]{origin} gt $1){
                    last
                }
            }            
        }
        if (/^GLOBAL SYMBOLS: SORTED BY Symbol Address\s*$/) {
            $armed = 1;
        }
        elsif ($armed) {
            if (/^[\da-f]{8}\s/i) {
                my @words = split /\s+/, $_;
                print CMM "symbol.create.label $words[1] 0x$words[0]\n";
            }
            #if (/^4020([a-z\d]{4})\s/) {
            if (/^([a-z\d]{8})\s/) {
                my $o = eval("0x$1");
                if (defined $offset) {
                    my $size = $o - $offset;
                    my $s = sprintf ("%04x  ",$size);
                    $lines{$s} .= "$s$line";
                }
                $offset = $o;
                $line = $_;
            }
        }
        print OUT $_;
    }
    print CMM "symbol.create.done\n";
    close OUT;
    close CMM;
    close IN;
    
    #$out = "$file.map.out";
    #open IN, $in || die $!;
    #open OUT, ">$out" || die $!;
    #while (<IN>){
    #    print OUT, $_;
    #}    
    #close OUT;
    #close IN;

    for my $line (sort keys %lines) {
        print $lines{$line}
    }
}

if (!@ARGV) {
    #read_map $dir.q(3430\Debug\dnld_startup_3430sdp_gp_romapi);
    #read_map $dir.q(flashdrv\Debug\nor_intel_sibley_drv);
    #read_map $dir.q(flashdrv\Debug\emmc_drv);
    read_map $dir.q(3630\Debug\dnld_startup_3630sdp_hs_hynix_4g);
}
else {    
    read_map $ARGV[0];
}
print ("DONE\n");