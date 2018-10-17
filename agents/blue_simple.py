import simpleagent

b = simpleagent.SimpleAgent('blue_sphero', opponent='red_sphero')

b.setup_ros()

b.play_game()
