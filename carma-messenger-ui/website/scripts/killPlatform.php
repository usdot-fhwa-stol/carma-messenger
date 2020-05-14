<?php
  // Kill the docker system
  # shell_exec("/var/www/html/scripts/kill.bash");
  shell_exec("/home/dandu/carma_messenger_ws/src/carma-messenger/carma-messenger-ui/website/scripts/kill.bash");
  // Move to index page
  header("Location: ../index.html"); // Move onto main.html
?>