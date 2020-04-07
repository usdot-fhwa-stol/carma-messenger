<?php
echo shell_exec ("docker container inspect -f '{{ index .Config.Labels \"org.label-schema.version\"}}'  carma-messenger-config"); 
?>



