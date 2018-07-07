import urllib.request
import urllib.error
distance = 16
angle = 10
fire = 1
urllib.request.urlopen(f"http://192.168.96.1:8080/update_cmd?distance={distance}&angle={angle}&fire={fire}")