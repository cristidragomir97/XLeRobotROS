#!/usr/bin/env python3

import select
import subprocess
import time
import threading
import traceback

class DMesgSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.process = None
        threading.Thread(target = self.start, daemon = True).start()

    def __del__(self):
        if self.process:
            self.process.terminate()
            self.process = None

    def unregister(self):
        if self.process:
            self.process.terminate()
            self.process = None

    def start(self):
        try:
            process = subprocess.Popen(['dmesg', '--follow'], stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT)
            self.process = process
            poller = select.poll()
            poller.register(process.stdout, select.POLLIN | select.POLLHUP)
            while self.process is process:
                if not poller.poll(100):
                    continue
                line = process.stdout.readline()
                if not line:
                    break
                self.callback(line.decode('utf-8', errors='replace').strip())
        except Exception as error:
            self.callback('Kernel log unavailable: ' + str(error))


if __name__ == "__main__":
    # Run test
    DMesgSubscriber(lambda msg: print("Received msg: %s" % msg))
    time.sleep(100)
