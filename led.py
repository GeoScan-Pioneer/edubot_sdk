import board
import neopixel
import time
import threading

class Led:
    def __init__(self):
        self.pixels = neopixel.NeoPixel(board.D18, 10)
        self.pixels.fill((255, 0, 0))
        time.sleep(1)
        self.pixels.fill((0, 0, 0))

        self.r = 0
        self.g = 0
        self.b = 0

        self.color1 = (0, 0, 0)
        self.color2 = (0, 0, 0)

        self.new_led_msg = False
        self.mode = 0
        self.led_time = 0
        self.__flash_period = 0.35

        self.led_thread = threading.Thread(target=self.__led_handler)
        self.led_thread.start()

    def __del__(self):
        self._set_color_rgb(0,0,0)
        pass

    def _set_color_rgb(self, r=0, g=0, b=0):
        self.pixels.fill((r, g, b))  # то, что было записано в буфер

    def _set_color_1(self):
        self._set_color_rgb(r=self.color1[0],
                            g=self.color1[1],
                            b=self.color1[2])

    def _set_color_2(self):
        self._set_color_rgb(r=self.color2[0],
                            g=self.color2[1],
                            b=self.color2[2])

    def handler_led_control(self, msg):
        self.mode = 1
        r = int(msg.param2)
        g = int(msg.param3)
        b = int(msg.param4)
        self.color2 = (r,g,b)
        self.color1 = (r,g,b)
        print(f"NEW COLOR: {r}, {g}, {b}")
        self.new_led_msg = True

    def handler_led_custom(self, msg):
        self.mode = int(msg.param5)
        if self.mode:
            self.new_led_msg = True
            r1 = (int(msg.param2) >> 16) & 0xff
            g1 = (int(msg.param2) >> 8) & 0xff
            b1 = int(msg.param2) & 0xff

            r2 = (int(msg.param3) >> 16) & 0xff
            g2 = (int(msg.param3) >> 8) & 0xff
            b2 = int(msg.param3) & 0xff
            self.color1 = (r1, g1, b1)
            self.color2 = (r2, g2, b2)
            self.led_time = int(msg.param6)
            self.new_led_msg = True
        else:
            self.r = int(msg.param2)
            self.g = int(msg.param3)
            self.b = int(msg.param4)


    def __led_handler(self):  # worked in thread
        state = False
        timer = 0
        flash_timer = 0
        disabled = False
        while True:
            curr_time = time.time()
            if self.new_led_msg:
                self.new_led_msg = False
                disabled = False
                timer = curr_time
                if self.mode == 2:
                    flash_timer = curr_time
            else:
                if self.mode == 1 and not disabled:
                    self._set_color_2()
                    disabled = True
                elif self.mode == 2:
                    if curr_time - flash_timer >= self.__flash_period:
                        state = not state
                        flash_timer = curr_time
                    if not state:
                        self._set_color_1()
                    else:
                        self._set_color_2()
                elif self.mode == 3:
                    self._set_color_1()
            if not (curr_time - timer <= self.led_time) and self.mode > 1 and self.led_time: # or  self.__heartbeat_dead.is_set()
                self.mode = 1
            time.sleep(0.01)
