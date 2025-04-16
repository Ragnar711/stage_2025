# Git

## Add SSH Key to your repository

- Create ssh key: `ssh-keygen -t rsa -f ~/.ssh/{KEY_FILENAME}`
- Read your ssh_key in the terminal: `cat ~/.ssh/{KEY_FILENAME}.pub`
- In **preferences** on gitlab, go to **SSH keys** and copy the ssh_key obtain before in *Key*.
- Click on **add key**.

## Create Access Tokens

- Go to **Access Tokens** and create a personal access token with all rights
- Save your token

## Clone your repository

- `git clone {URL_REPOSITORY}` on the git project page, click on **clone** and get the *https url_repository* on clone with HTTPS.
- Authentification with the gitlab username and the personal access token

## Branch

- Fetch the contents of the repository and download it, then immediately update the local repository that matches those contents : `git pull` (Do not forget to do it: get the updated code, or new branches created)
- List the different branches: `git branch -a`
- Go to specific branch: `git checkout name_branch`
- Check the branch you are on: `git status`
