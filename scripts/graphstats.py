#!/usr/bin/env python2
# Script to parse a logging file, extract the stats, and graph them
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse, datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot, matplotlib.dates, matplotlib.font_manager
import matplotlib.ticker

MAXBANDWIDTH=25000.
MAXBUFFER=2.
STATS_INTERVAL=5.
TASK_MAX=0.0025

APPLY_PREFIX = ['mcu_awake', 'mcu_task_avg', 'mcu_task_stddev', 'bytes_write',
                'bytes_read', 'bytes_retransmit', 'freq', 'adj']

def parse_log(logname, mcu):
    if mcu is None:
        mcu = "mcu"
    mcu_prefix = mcu + ":"
    apply_prefix = { p: 1 for p in APPLY_PREFIX }
    f = open(logname, 'rb')
    out = []
    for line in f:
        parts = line.split()
        if not parts or parts[0] not in ('Stats', 'INFO:root:Stats'):
            #if parts and parts[0] == 'INFO:root:shutdown:':
            #    break
            continue
        prefix = ""
        keyparts = {}
        for p in parts[2:]:
            if '=' not in p:
                prefix = p
                if prefix == mcu_prefix:
                    prefix = ''
                continue
            name, val = p.split('=', 1)
            if name in apply_prefix:
                name = prefix + name
            keyparts[name] = val
        if keyparts.get('bytes_write', '0') == '0':
            continue
        keyparts['#sampletime'] = float(parts[1][:-1])
        out.append(keyparts)
    f.close()
    return out

def find_print_restarts(data):
    runoff_samples = {}
    last_runoff_start = last_buffer_time = last_sampletime = 0.
    last_print_stall = 0
    for d in reversed(data):
        # Check for buffer runoff
        sampletime = d['#sampletime']
        buffer_time = float(d.get('buffer_time', 0.))
        if (last_runoff_start and last_sampletime - sampletime < 5
            and buffer_time > last_buffer_time):
            runoff_samples[last_runoff_start][1].append(sampletime)
        elif buffer_time < 1.:
            last_runoff_start = sampletime
            runoff_samples[last_runoff_start] = [False, [sampletime]]
        else:
            last_runoff_start = 0.
        last_buffer_time = buffer_time
        last_sampletime = sampletime
        # Check for print stall
        print_stall = int(d['print_stall'])
        if print_stall < last_print_stall:
            if last_runoff_start:
                runoff_samples[last_runoff_start][0] = True
        last_print_stall = print_stall
    sample_resets = {sampletime: 1 for stall, samples in runoff_samples.values()
                     for sampletime in samples if not stall}
    return sample_resets

def plot_mcu(data, maxbw, outname):
    # Generate data for plot
    basetime = lasttime = data[0]['#sampletime']
    lastbw = float(data[0]['bytes_write']) + float(data[0]['bytes_retransmit'])
    sample_resets = find_print_restarts(data)
    times = []
    bwdeltas = []
    loads = []
    awake = []
    hostbuffers = []
    for d in data:
        st = d['#sampletime']
        timedelta = st - lasttime
        if timedelta <= 0.:
            continue
        bw = float(d['bytes_write']) + float(d['bytes_retransmit'])
        if bw < lastbw:
            lastbw = bw
            continue
        load = float(d['mcu_task_avg']) + 3*float(d['mcu_task_stddev'])
        if st - basetime < 15.:
            load = 0.
        pt = float(d['print_time'])
        hb = float(d['buffer_time'])
        if hb >= MAXBUFFER or st in sample_resets:
            hb = 0.
        else:
            hb = 100. * (MAXBUFFER - hb) / MAXBUFFER
        hostbuffers.append(hb)
        times.append(datetime.datetime.utcfromtimestamp(st))
        bwdeltas.append(100. * (bw - lastbw) / (maxbw * timedelta))
        loads.append(100. * load / TASK_MAX)
        awake.append(100. * float(d.get('mcu_awake', 0.)) / STATS_INTERVAL)
        lasttime = st
        lastbw = bw

    # Build plot
    fig, ax1 = matplotlib.pyplot.subplots()
    ax1.set_title("MCU bandwidth and load utilization")
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Usage (%)')
    ax1.plot_date(times, bwdeltas, 'g', label='Bandwidth', alpha=0.8)
    ax1.plot_date(times, loads, 'r', label='MCU load', alpha=0.8)
    ax1.plot_date(times, hostbuffers, 'c', label='Host buffer', alpha=0.8)
    ax1.plot_date(times, awake, 'y', label='Awake time', alpha=0.6)
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    ax1.legend(loc='best', prop=fontP)
    ax1.xaxis.set_major_formatter(matplotlib.dates.DateFormatter('%H:%M'))
    ax1.grid(True)
    fig.set_size_inches(8, 6)
    fig.savefig(outname)

def plot_frequency(data, outname, mcu):
    all_keys = {}
    for d in data:
        all_keys.update(d)
    one_mcu = mcu is not None
    graph_keys = { key: ([], []) for key in all_keys
                   if (key in ("freq", "adj") or (not one_mcu and (
                           key.endswith(":freq") or key.endswith(":adj")))) }
    basetime = lasttime = data[0]['#sampletime']
    for d in data:
        st = datetime.datetime.utcfromtimestamp(d['#sampletime'])
        for key, (times, values) in graph_keys.items():
            val = d.get(key)
            if val not in (None, '0', '1'):
                times.append(st)
                values.append(float(val))

    # Build plot
    fig, ax1 = matplotlib.pyplot.subplots()
    if one_mcu:
        ax1.set_title("MCU '%s' frequency" % (mcu,))
    else:
        ax1.set_title("MCU frequency")
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Frequency')
    for key in sorted(graph_keys):
        times, values = graph_keys[key]
        ax1.plot_date(times, values, '.', label=key)
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    ax1.legend(loc='best', prop=fontP)
    ax1.xaxis.set_major_formatter(matplotlib.dates.DateFormatter('%H:%M'))
    ax1.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter('%d'))
    ax1.grid(True)
    fig.set_size_inches(8, 6)
    fig.savefig(outname)

def main():
    usage = "%prog [options] <logfile> <outname>"
    opts = optparse.OptionParser(usage)
    opts.add_option("-f", "--frequency", action="store_true",
                    help="graph mcu frequency")
    opts.add_option("-m", "--mcu", type="string", dest="mcu", default=None,
                    help="limit stats to the given mcu")
    options, args = opts.parse_args()
    if len(args) != 2:
        opts.error("Incorrect number of arguments")
    logname, outname = args
    data = parse_log(logname, options.mcu)
    if not data:
        return
    if options.frequency:
        plot_frequency(data, outname, options.mcu)
        return
    plot_mcu(data, MAXBANDWIDTH, outname)

if __name__ == '__main__':
    main()
