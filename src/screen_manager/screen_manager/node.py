import time
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node
import sys

POS_ACTIVE = (1, 0.0, 0.9)
POS_CACHE  = (0.0, 0.0, -5.0)
OBJETS = ["video_screen_0", "video_screen_1", "video_screen_2"]

def deplacer(node, nom, position):
    service = "/world/default/set_pose"
    msg = Pose()
    msg.name = nom
    msg.position.x, msg.position.y, msg.position.z = position
    msg.orientation.w = 1.0
    
    ok, response = node.request(service, msg, Pose, Boolean, 1000)
    if ok:
        print(f"{nom} moved.")
    else:
        print(f"Error with {nom}")
    return ok

def afficher_uniquement(node, index_a_afficher):
    for i, nom in enumerate(OBJETS):
        pos = POS_ACTIVE if i == index_a_afficher else POS_CACHE
        deplacer(node, nom, pos)

def main():
    node = Node()
    print("\n--- MANUAL CONTROL ---")
    print("Type 0, 1, or 2 and press Enter to change the image.")
    print("Type 'q' to quit.\n")

    while True:
        choix = input("Choice (0/1/2/q): ")
        
        if choix == 'q':
            break
        elif choix in ['0', '1', '2']:
            afficher_uniquement(node, int(choix))
        else:
            print("Unrecognized key.")

if __name__ == "__main__":
    main()