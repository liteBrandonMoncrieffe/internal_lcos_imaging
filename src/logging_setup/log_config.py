import logging
import sys
from pathlib import Path


def setup_logging(log_level=logging.INFO, log_file=None):
    """
    Configure logging with standard settings.
    
    Args:
        log_level: Logging level (default: logging.INFO)
        log_file: Optional path to log file. If None, logs to console only.
    """
    # Standard format with date, time, level, logger name, and message
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    date_format = '%Y-%m-%d %H:%M:%S'
    
    # Configure root logger
    logging.basicConfig(
        level=log_level,
        format=log_format,
        datefmt=date_format,
        handlers=[]
    )
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(logging.Formatter(log_format, datefmt=date_format))
    logging.getLogger().addHandler(console_handler)
    
    # Optional file handler
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(log_level)
        file_handler.setFormatter(logging.Formatter(log_format, datefmt=date_format))
        logging.getLogger().addHandler(file_handler)
    
    logging.info("Logging configured successfully")
    
    return logging.getLogger()
