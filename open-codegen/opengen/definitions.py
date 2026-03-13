from importlib.resources import files, as_file


def templates_dir():
    """Directory where the templates are found (for internal use, mainly)"""
    return files("opengen") / "templates"


def templates_subdir(subdir=None):
    """
    Directory where the templates are found and subfolder relative
    to that path (for internal use, mainly)
    """
    resource = files("opengen") / "templates"
    if subdir is not None:
        resource = resource / subdir
    return resource


def original_icasadi_dir():
    """Directory where the original icasadi files are found (for internal use)"""
    return files("opengen") / "icasadi"
