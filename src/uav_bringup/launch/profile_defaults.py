WORLD_PROFILES = {
    "test": {
        "ground_height": "-0.30",
    },
    "living_room": {
        "ground_height": "-0.30",
    },
    "baylands": {
        "ground_height": "-0.30",
    },
}


def _quote(value):
    return '"' + str(value) + '"'


def profile_expression(profile_config, selector, key):
    expr = []
    items = list(profile_config.items())

    for index, (name, values) in enumerate(items):
        if index == 0:
            expr.extend([_quote(values[key]), ' if "', selector, '" == "', name, '"'])
        else:
            expr.extend([' else ', _quote(values[key]), ' if "', selector, '" == "', name, '"'])

    expr.extend([' else ', _quote(items[0][1][key])])
    return expr
