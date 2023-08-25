source scripts/constant.sh


if [ ! -d $public_dir ]; then
    mkdir -p $public_dir
fi

to_be_download=(
    "https://files.pythonhosted.org/packages/07/bc/587a445451b253b285629263eb51c2d8e9bcea4fc97826266d186f96f558/pyserial-3.5-py2.py3-none-any.whl"
    "https://files.pythonhosted.org/packages/41/f4/a7ca4859317232b1efb64a826b8d2d7299bb77fb60bdb08e2bd1d61cf80d/setuptools-58.2.0-py3-none-any.whl"
)


for url in ${to_be_download[@]}; do
    wget -P $public_dir $url
done