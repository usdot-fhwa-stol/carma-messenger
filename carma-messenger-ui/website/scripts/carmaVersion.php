<?php
echo exec ("docker container inspect -f '{{ index .Config.Labels \"org.label-schema.version\"}}' carma-config");
?>



