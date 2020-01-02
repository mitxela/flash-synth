
VERSION="00000003" # hex

if [ `git diff-index --quiet HEAD --` ]; then
  HASH=$(printf "0%s" $(git rev-parse --short HEAD))
else 
  HASH=$(printf '%x\n' $(date +%s))
fi

echo "001C: $VERSION $HASH" | xxd -r - $1
