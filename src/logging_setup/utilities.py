from types import SimpleNamespace

def to_namespace(data):
    """Convert JSON dict to SimpleNamespace for dot notation access."""
    if isinstance(data, dict):
        return SimpleNamespace(**{k: to_namespace(v) for k, v in data.items()})
    elif isinstance(data, list):
        return [to_namespace(item) for item in data]
    else:
        return data

def wait_for_user_action(message):
    """
    Display a message and wait for user confirmation in the console.
    """
    input(f"{message} (Press Enter to continue)")
