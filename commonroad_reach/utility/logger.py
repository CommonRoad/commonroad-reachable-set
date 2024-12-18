import logging
import os
from datetime import datetime


def initialize_logger(config) -> logging.Logger:
    """
    Initializes the logging module and returns a logger.
    """
    # create log directory
    os.makedirs(config.general.path_logs, exist_ok=True)

    # create logger
    logger = logging.getLogger()

    # create file handler (outputs to file)
    string_date_time = datetime.now().strftime("_%Y_%m_%d_%H-%M-%S")
    path_log = os.path.join(config.general.path_logs, f"{config.scenario.scenario_id}{string_date_time}.log")
    file_handler = logging.FileHandler(path_log)

    # set logging levels
    logger.setLevel(logging.DEBUG)
    file_handler.setLevel(logging.DEBUG)

    # create log formatter
    # formatter = logging.Formatter('%(asctime)s\t%(filename)s\t\t%(funcName)s@%(lineno)d\t%(levelname)s\t%(message)s')
    formatter = logging.Formatter("%(levelname)-8s [%(asctime)s] --- %(message)s (%(filename)s:%(lineno)s)",
                                  "%Y-%m-%d %H:%M:%S")
    file_handler.setFormatter(formatter)

    # add handlers
    logger.addHandler(file_handler)

    return logger


def print_and_log_debug(logger: logging.Logger, message: str, verbose: bool = False):
    if verbose:
        print(message)
    logger.debug(message)


def print_and_log_info(logger: logging.Logger, message: str, verbose: bool = True):
    if verbose:
        print(message)
    logger.info(message)


def print_and_log_warning(logger: logging.Logger, message: str, verbose: bool = True):
    if verbose:
        print(message)
    logger.warning(message)


def print_and_log_error(logger: logging.Logger, message: str, verbose: bool = True):
    if verbose:
        print(message)
    logger.error(message)
