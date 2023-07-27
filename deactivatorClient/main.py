# -*- coding: utf-8 -*-
from DeactivatorCli import DeactivatorCli
import argparse



parser = argparse.ArgumentParser(description='Debug Client (press q to exit)')
parser.add_argument('-d', dest='device', default='/dev/ttyUSB1', help='UART Device')
parser.add_argument('-b', dest='bauds',  default=115200, type=int,
                    help='Bauds')

args = parser.parse_args()



_cli = DeactivatorCli(args.device, args.bauds)


repeat = True

while repeat:
    repeat = _cli.options()

del _cli




