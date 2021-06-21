def get_data(self, key, required=False):
    """
        This function returns value in self.event against a specified 'key'.

        Hierarchy levels in a nested key are separated by '.' (dot). If required is True and key is not found or is
        empty, the function will raise an exception. If required is False and key is not found or is empty, the
        function will log the instance.

                Parameters:
                        key (str): dotted-separated key string
                        required (bool) : whether or not key is a required field (default: None)
                Returns:
                        ANY (str) : value of key in self.event or 'None' if key is not found or is empty
                Raises:
                        ValueError : If key is not found or is empty and required is True
                Example:
                        key = 'key_a.key_b.key_c' will return 'value_abc' if:
                        self.event = {
                            'key_a': {
                                'key_b': {
                                    'key_c': 'value_abc'
                                }
                            }
                        }
        """

    result = get_value_from_dotted_path(self.event, key)
    if result is None:
        if not required:
            logger.info('Could not get data for %s in event "%s"', key, self.get_data('name', True))
        else:
            raise ValueError(
                'Could not get data for {} in event "{}"'.format(key, self.get_data('name', True))
            )

    return result
