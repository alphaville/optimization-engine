import pkg_resources


def templates_dir():
    """Directory where the templates are found (for internal use, mainly)"""
    return pkg_resources.resource_filename('opengen', 'templates/')


def templates_subdir(subdir=None):
    if subdir is None:
        return templates_dir()
    else:
        return pkg_resources.resource_filename('opengen', 'templates/%s/' % subdir)


def original_icasadi_dir():
    """Directory where the original icasadi files are found (for internal use)"""
    return pkg_resources.resource_filename('opengen', 'icasadi/')
