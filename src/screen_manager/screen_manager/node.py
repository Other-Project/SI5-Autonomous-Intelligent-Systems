import time
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node
import sys

POS_ACTIVE = (1, 0.0, 0.9)
POS_CACHE  = (0.0, 0.0, -5.0)
OBJETS = ["video_screen_0", "video_screen_1", "video_screen_2"]

# Mapping pour la lisibilité
MAPPING = {
    '0': "NORMAL",
    '1': "START FOLLOWING",
    '2': "STOP"
}

def deplacer(node, nom, position):
    service = "/world/default/set_pose"
    msg = Pose()
    msg.name = nom
    msg.position.x, msg.position.y, msg.position.z = position
    msg.orientation.w = 1.0
    
    ok, response = node.request(service, msg, Pose, Boolean, 1000)
    if ok:
        # On affiche le nom correspondant au mapping si possible
        suffixe = nom.split('_')[-1]
        action = MAPPING.get(suffixe, nom)
        print(f"Status changed to: {action}")
    else:
        print(f"Error with {nom}")
    return ok

def afficher_uniquement(node, index_a_afficher):
    for i, nom in enumerate(OBJETS):
        pos = POS_ACTIVE if i == index_a_afficher else POS_CACHE
        deplacer(node, nom, pos)

def main():
    node = Node()
    print("\n--- ROBOT CONTROL INTERFACE ---")
    print("Commands:")
    print("  0 : NORMAL")
    print("  1 : START FOLLOWING")
    print("  2 : STOP")
    print("  q : Quit\n")

    while True:
        choix = input("Choice (0/1/2/q): ").strip()
        
        if choix.lower() == 'q':
            print("Exiting...")
            break
        elif choix in MAPPING:
            print(f"\nTriggering: {MAPPING[choix]}")
            afficher_uniquement(node, int(choix))
        else:
            print(f"Invalid choice '{choix}'. Please use 0, 1, 2 or q.")

if __name__ == "__main__":
    main()