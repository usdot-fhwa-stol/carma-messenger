<?php
  // Kill the docker system
  shell_exec("/var/www/html/scripts/kill.bash");
  // Move to index page
  header("Location: ../index.html"); 
?>