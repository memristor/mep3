#!/usr/bin/env fish

if status --is-interactive
    set -g fish_user_abbreviations
    # Git shortcuts
    abbr --add gs   'git status'
    abbr --add gsh  'git show'
    abbr --add ga   'git add'
    abbr --add gc   'git commit -m'
    abbr --add gca  'git commit --amend'
    abbr --add gl   'git log'
    abbr --add gp   'git push'
    abbr --add gpf  'git push --force'
    abbr --add gw   'git whatchanged'
    abbr --add gm   'git merge'
    abbr --add gpl  'git pull'
    abbr --add gplr 'git pull --rebase'
    abbr --add gco  'git checkout'
    abbr --add gb   'git branch'
    abbr --add gsw  'git switch'
    abbr --add gr   'git restore'
    abbr --add grb  'git rebase --interactive --committer-date-is-author-date'
end
