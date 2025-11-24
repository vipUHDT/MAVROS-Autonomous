# --- set where those folders live ---
BASE="ros2_ws/src/"   # <-- change to "." if they’re at repo root

# sanity check
ls -la "$BASE"

# list of submodule folder names (no leading path)
mods=(Livox-SDK2 examples livox_ros_driver2 mavros)

# 1) detach each submodule (remove gitlink) but keep files on disk
for name in "${mods[@]}"; do
  path="$BASE/$name"
  echo "Detaching submodule: $path"
  git rm -r --cached "$path" 2>/dev/null || true
  rm -rf ".git/modules/$path" 2>/dev/null || true
  rm -rf "$path/.git" 2>/dev/null || true
  # remove its section from .gitmodules if present
  [ -f .gitmodules ] && git config -f .gitmodules --remove-section "submodule.$path" 2>/dev/null || true
done

# 2) drop .gitmodules if now empty
[ -f .gitmodules ] && [ ! -s .gitmodules ] && git rm .gitmodules || true

# 3) re-add as normal content
git add "$BASE"
git commit -m "Vendor deps: convert $BASE/* from submodules to regular directories"
git push
