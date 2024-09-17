#!/bin/bash
set -e

start=`date +%s`
auterion-cli device select --ip 10.41.1.1

# path of files / directories to sync, relative to root of container
# should include all files/folders required by the docker container at runtime.
items_to_sync=(
    /opt/sysroot/app/build
)

app_name=`grep 'app-name:' auterion-app.yml | cut -d ":" -f 2 | xargs`
app_author=`grep 'app-author:' auterion-app.yml | cut -d ":" -f 2 | xargs`
app_version_local=`grep 'app-version:' auterion-app.yml | cut -d ":" -f 2 | xargs`
app_name_full=$app_author.$app_name
container_name=$app_name_full.$app_name
app_version_remote=`auterion-cli app version $app_name_full`

echo "Full app name: $app_author.$app_name"
echo "Local version: $app_version_local"
echo "Remote version: $app_version_remote"
echo "Container name: $container_name"

echo "App version on device is up to date. Incremental update"
auterion-cli app build --skip-packaging
docker rm -f tmp_container || true
docker create --name tmp_container $container_name:$app_version_local
rm -rf build/incremental_update
mkdir -p build/incremental_update

for item in ${items_to_sync[@]}; do
    docker cp tmp_container:$item build/incremental_update/
done

docker rm -f tmp_container

ssh root@10.41.1.1 "rm -rf /tmp/$app_name_full && mkdir -p /tmp/$app_name_full"
scp -r build/incremental_update/* root@10.41.1.1:/tmp/$app_name_full
ssh root@10.41.1.1 "cd /tmp/$app_name_full && tar -cf - . | docker exec -i $container_name sh -c 'tar -xf - -C /'"
ssh root@10.41.1.1 "docker cp /tmp/$app_name_full/build/precision_landing $container_name:/opt/sysroot/app/build/precision_landing"
ssh root@10.41.1.1 "docker commit $container_name $container_name:$app_version_local"

# auterion-cli app restart $app_name_full

ssh root@10.41.1.1 "rm -rf /tmp/$app_name_full"

end=`date +%s`
runtime=$((end-start))
echo "=== updating $app_name_full took $runtime s ==="