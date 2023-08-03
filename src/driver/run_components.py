# -*- coding: utf-8 -*-

import logging
import os
import os.path
import subprocess
import sys
import shutil
from . import limits

from . import portfolio_runner
from .plan_manager import PlanManager


DRIVER_DIR = os.path.abspath(os.path.dirname(__file__))
SRC_DIR = os.path.dirname(DRIVER_DIR)
TRANSLATE = os.path.join(SRC_DIR, "translate-fond", "translate.py")
PREPROCESS = os.path.join(SRC_DIR, "preprocess", "preprocess")
SEARCH_DIR = os.path.join(SRC_DIR, "search")


def call_cmd(cmd, args, debug, stdin=None):
    if not os.path.exists(cmd):
        target = " debug" if debug else ""
        raise IOError(
            "Could not find %s. Please run \"./build_all%s\"." %
            (cmd, target))
    sys.stdout.flush()
    if stdin:
        with open(stdin) as stdin_file:
            subprocess.check_call([cmd] + args, stdin=stdin_file)
    else:
        subprocess.check_call([cmd] + args)


def run_translate(args):
    logging.info("Running translator.")
    logging.info("translator inputs: %s" % args.translate_inputs)
    logging.info("translator arguments: %s" % args.translate_options)
    call_cmd(TRANSLATE, args.translate_inputs + args.translate_options,
             debug=args.debug)


def run_preprocess(args):
    logging.info("No preprocessor in the FOND version.")
    logging.info("Copying %s into %s." % (args.preprocess_input, args.search_input))
    shutil.copy2(args.preprocess_input, args.search_input)

    #logging.info("Running preprocessor.")
    #logging.info("preprocessor input: %s" % args.preprocess_input)
    #logging.info("preprocessor arguments: %s" % args.preprocess_options)
    #call_cmd(PREPROCESS, args.preprocess_options, debug=args.debug,
    #         stdin=args.preprocess_input)


def run_search(args):
    logging.info("Running search.")
    logging.info("search input: %s" % args.search_input)
   
    time_limit = limits.get_time_limit(
        args.search_time_limit, args.overall_time_limit)
    memory_limit = limits.get_memory_limit(
        args.search_memory_limit, args.overall_memory_limit)
    print_component_settings(
        "search", args.search_input, args.search_options,
         time_limit, memory_limit)
    
    plan_manager = PlanManager(args.plan_file)
    plan_manager.delete_existing_plans()

    if args.debug:
        executable = os.path.join(SEARCH_DIR, "downward-debug")
    else:
        executable = os.path.join(SEARCH_DIR, "downward-release")
    logging.info("search executable: %s" % executable)

    if args.portfolio:
        assert not args.search_options
        logging.info("search portfolio: %s" % args.portfolio)
        portfolio_runner.run(
            args.portfolio, executable, args.search_input, plan_manager)
    else:
        if not args.search_options:
            raise ValueError(
                "search needs --alias, --portfolio, or search options")
        if "--help" not in args.search_options:
            args.search_options.extend(["--internal-plan-file", args.plan_file])
        logging.info("search arguments: %s" % args.search_options)
        call_cmd(executable, args.search_options, debug=args.debug,
                 stdin=args.search_input)

def print_component_settings(nick, inputs, options, time_limit, memory_limit):
    logging.info("{} input: {}".format(nick, inputs))
    logging.info("{} arguments: {}".format(nick, options))
    if time_limit is not None:
        time_limit = str(time_limit) + "s"
    logging.info("{} time limit: {}".format(nick, time_limit))
    if memory_limit is not None:
        memory_limit = int(limits.convert_to_mb(memory_limit))
        memory_limit = str(memory_limit) + " MB"
    logging.info("{} memory limit: {}".format(nick, memory_limit))


