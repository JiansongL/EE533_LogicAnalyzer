#!/usr/bin/perl -w
use strict;
use warnings;
use FindBin;
use lib "$FindBin::Bin/../lib/Perl5";

use reg_defines_lab6;

my $REG_DMEM_ADDR        = PIPELINE_PROCESSOR_REG_DMEM_ADDR_REG();
my $REG_PIPELINE_C       = PIPELINE_PROCESSOR_REG_PIPELINE_C_REG();
my $REG_IMEM_OUT         = PIPELINE_PROCESSOR_REG_IMEM_OUT_REG();
my $REG_INST_ADDR_LO     = PIPELINE_PROCESSOR_REG_INST_ADDR_LO_REG();
my $REG_INST_ADDR_HI     = PIPELINE_PROCESSOR_REG_INST_ADDR_HI_REG();
my $REG_WB_RES_LO        = PIPELINE_PROCESSOR_REG_WB_RES_LO_REG();
my $REG_WB_RES_HI        = PIPELINE_PROCESSOR_REG_WB_RES_HI_REG();
my $REG_PIPELINE_STATUS  = PIPELINE_PROCESSOR_REG_PIPELINE_STATUS_REG();

my $BIT_IMEM_WE      = 0;
my $BIT_TRACE_EN     = 1;
my $BIT_TRACE_CLEAR  = 2;
my $BIT_TRACE_FREEZE = 3;
my $BIT_TRACE_VIEW   = 4;

sub usage {
    print "Usage: logic_analyzer_regs.pl <command> [args]\n";
    print "\n";
    print "Commands:\n";
    print "  status                    Show decoded pipeline/trace status\n";
    print "  live                      Show live InstADDR and WB_Res\n";
    print "  clear                     Pulse trace clear\n";
    print "  start                     Enable trace capture\n";
    print "  stop                      Disable trace capture\n";
    print "  freeze on|off             Control trace freeze bit\n";
    print "  view on|off               Control trace view mode bit\n";
    print "  sample <idx>              Read one trace sample (0-63)\n";
    print "  dump [count] [start_idx]  Dump trace samples (default 64 from 0)\n";
    print "\n";
    print "Notes:\n";
    print "  - sample/dump force view mode ON while reading.\n";
    print "  - To capture meaningful history: clear -> start -> run -> freeze on -> dump.\n";
}

sub run_cmd {
    my ($cmd) = @_;
    my @out = qx{$cmd 2>&1};
    my $rc = $? >> 8;
    if ($rc != 0) {
        die "Command failed ($rc): $cmd\n" . join('', @out);
    }
    return @out;
}

sub regwrite {
    my ($addr, $value) = @_;
    my $cmd = sprintf('regwrite 0x%08x 0x%08x', $addr, $value & 0xffffffff);
    run_cmd($cmd);
}

sub regread_u32 {
    my ($addr) = @_;
    my $cmd = sprintf('regread 0x%08x', $addr);
    my @out = run_cmd($cmd);
    my $line = $out[0] // '';

    # Typical format: Reg 0xXXXXXXXX (N): 0xYYYYYYYY (N)
    if ($line =~ /:\s*(0x[0-9a-fA-F]+)/) {
        return hex($1) & 0xffffffff;
    }
    die "Unable to parse regread output for address " . sprintf('0x%08x', $addr) . ": $line\n";
}

sub get_pipeline_c {
    return regread_u32($REG_PIPELINE_C);
}

sub set_pipeline_c {
    my ($value) = @_;
    regwrite($REG_PIPELINE_C, $value);
}

sub set_bit {
    my ($value, $bit, $on) = @_;
    if ($on) {
        return $value | (1 << $bit);
    }
    return $value & ~(1 << $bit);
}

sub pulse_trace_clear {
    my $c = get_pipeline_c();
    my $with_clear = set_bit($c, $BIT_TRACE_CLEAR, 1);
    set_pipeline_c($with_clear);
    my $without_clear = set_bit($with_clear, $BIT_TRACE_CLEAR, 0);
    set_pipeline_c($without_clear);
}

sub read_status_decoded {
    my $s = regread_u32($REG_PIPELINE_STATUS);
    my %d;
    $d{raw}            = $s;
    $d{trace_full}     = ($s >> 15) & 0x1;
    $d{trace_wrapped}  = ($s >> 14) & 0x1;
    $d{trace_freeze}   = ($s >> 13) & 0x1;
    $d{trace_enable}   = ($s >> 12) & 0x1;
    $d{trace_view_en}  = ($s >> 11) & 0x1;
    $d{trace_wr_ptr}   = ($s >> 5) & 0x3f;
    $d{proc_rst}       = ($s >> 4) & 0x1;
    $d{imem_write_en}  = ($s >> 3) & 0x1;
    $d{inst_wr_pulse}  = ($s >> 2) & 0x1;
    $d{trace_clear}    = ($s >> 1) & 0x1;
    return %d;
}

sub print_status {
    my %d = read_status_decoded();
    printf("pipeline_status raw   : 0x%08x\n", $d{raw});
    printf("  trace_full          : %u\n", $d{trace_full});
    printf("  trace_wrapped       : %u\n", $d{trace_wrapped});
    printf("  trace_freeze        : %u\n", $d{trace_freeze});
    printf("  trace_enable        : %u\n", $d{trace_enable});
    printf("  trace_view_en       : %u\n", $d{trace_view_en});
    printf("  trace_wr_ptr        : %u\n", $d{trace_wr_ptr});
    printf("  proc_rst            : %u\n", $d{proc_rst});
    printf("  imem_write_en       : %u\n", $d{imem_write_en});
    printf("  inst_write_pulse    : %u\n", $d{inst_wr_pulse});
    printf("  trace_clear         : %u\n", $d{trace_clear});
}

sub print_live {
    my $pc_lo = regread_u32($REG_INST_ADDR_LO);
    my $pc_hi = regread_u32($REG_INST_ADDR_HI);
    my $wb_lo = regread_u32($REG_WB_RES_LO);
    my $wb_hi = regread_u32($REG_WB_RES_HI);

    printf("InstADDR: 0x%08x%08x\n", $pc_hi, $pc_lo);
    printf("WB_Res  : 0x%08x%08x\n", $wb_hi, $wb_lo);
}

sub set_bool_control {
    my ($bit, $on) = @_;
    my $c = get_pipeline_c();
    $c = set_bit($c, $bit, $on);
    set_pipeline_c($c);
}

sub read_sample {
    my ($idx) = @_;
    if ($idx < 0 || $idx > 63) {
        die "Sample index must be in [0,63]\n";
    }

    my $orig_c = get_pipeline_c();
    my $view_c = set_bit($orig_c, $BIT_TRACE_VIEW, 1);
    set_pipeline_c($view_c);

    regwrite($REG_DMEM_ADDR, $idx);

    # One throwaway read for register-latency alignment.
    regread_u32($REG_INST_ADDR_LO);

    my $pc_lo = regread_u32($REG_INST_ADDR_LO);
    my $pc_hi = regread_u32($REG_INST_ADDR_HI);
    my $wb_lo = regread_u32($REG_WB_RES_LO);
    my $wb_hi = regread_u32($REG_WB_RES_HI);
    my $meta  = regread_u32($REG_IMEM_OUT);

    my $meta_rd_ptr   = $meta & 0x3f;
    my $meta_wrapped  = ($meta >> 6) & 0x1;
    my $meta_full     = ($meta >> 7) & 0x1;

    printf("idx=%02u  InstADDR=0x%08x%08x  WB_Res=0x%08x%08x  [meta rd_ptr=%u wrapped=%u full=%u]\n",
        $idx, $pc_hi, $pc_lo, $wb_hi, $wb_lo, $meta_rd_ptr, $meta_wrapped, $meta_full);
}

sub dump_samples {
    my ($count, $start_idx) = @_;
    $count = 64 if !defined($count);
    $start_idx = 0 if !defined($start_idx);

    if ($count < 1 || $count > 64) {
        die "count must be in [1,64]\n";
    }
    if ($start_idx < 0 || $start_idx > 63) {
        die "start_idx must be in [0,63]\n";
    }

    my $i;
    for ($i = 0; $i < $count; $i++) {
        my $idx = ($start_idx + $i) & 0x3f;
        read_sample($idx);
    }
}

sub cmd_start {
    set_bool_control($BIT_TRACE_EN, 1);
    print "Trace capture enabled.\n";
}

sub cmd_stop {
    set_bool_control($BIT_TRACE_EN, 0);
    print "Trace capture disabled.\n";
}

sub cmd_freeze {
    my ($arg) = @_;
    if (!defined($arg) || ($arg ne 'on' && $arg ne 'off')) {
        die "freeze requires argument on|off\n";
    }
    set_bool_control($BIT_TRACE_FREEZE, $arg eq 'on');
    print "Trace freeze set to $arg.\n";
}

sub cmd_view {
    my ($arg) = @_;
    if (!defined($arg) || ($arg ne 'on' && $arg ne 'off')) {
        die "view requires argument on|off\n";
    }
    set_bool_control($BIT_TRACE_VIEW, $arg eq 'on');
    print "Trace view mode set to $arg.\n";
}

my $argc = scalar(@ARGV);
if ($argc < 1) {
    usage();
    exit(1);
}

my $cmd = $ARGV[0];
if ($cmd eq 'status') {
    print_status();
} elsif ($cmd eq 'live') {
    print_live();
} elsif ($cmd eq 'clear') {
    pulse_trace_clear();
    print "Trace buffer clear pulse issued.\n";
} elsif ($cmd eq 'start') {
    cmd_start();
} elsif ($cmd eq 'stop') {
    cmd_stop();
} elsif ($cmd eq 'freeze') {
    cmd_freeze($ARGV[1]);
} elsif ($cmd eq 'view') {
    cmd_view($ARGV[1]);
} elsif ($cmd eq 'sample') {
    if (!defined($ARGV[1])) {
        die "sample requires <idx>\n";
    }
    read_sample(int($ARGV[1]));
} elsif ($cmd eq 'dump') {
    my $count = defined($ARGV[1]) ? int($ARGV[1]) : 64;
    my $start = defined($ARGV[2]) ? int($ARGV[2]) : 0;
    dump_samples($count, $start);
} else {
    print "Unknown command: $cmd\n";
    usage();
    exit(1);
}
